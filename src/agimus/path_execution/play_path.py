#!/usr/bin/env python
import rospy
import smach
import smach_ros
import std_srvs.srv
from agimus_hpp import ros_tools
from agimus_hpp.client import HppClient
from agimus_sot_msgs.msg import *
from agimus_sot_msgs.srv import *
from std_msgs.msg import Empty, Int32, String, UInt32


_outcomes = ["succeeded", "aborted", "preempted"]


## Waits if the step by step level is lower than level
#
# The step by step level is stored in ROS param `step_by_step`.
#  \param msg human readable message.
#  \param level 0 means *always wait*.
#  \param time to wait **after** the message is received.
#
# \todo It should be possible to handle errors while waiting for user input.
def wait_if_step_by_step(msg, level, time=0.1):
    l = rospy.get_param("step_by_step", 0)
    if type(l) == bool:
        l = 10 if l else 0
    if level < l:
        rospy.loginfo(
            "{} Wait for message on {}/step.".format(msg, rospy.get_namespace())
        )
        # rospy.wait_for_message (rospy.get_namespace() + "/step", Empty)
        rospy.wait_for_message("step", Empty)
        rospy.sleep(time)


class ErrorEvent(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


## Initialize the trajectory publisher.
#
# See agimus_hpp.trajectory_publisher.HppOutputQueue
class InitializePath(smach.State):
    hppTargetPubDict = {"read_subpath": [ReadSubPath, 1]}
    serviceProxiesDict = {
        "agimus": {"sot": {"clear_queues": [std_srvs.srv.Trigger]}},
        "hpp": {"target": {"publish_first": [std_srvs.srv.Trigger]}},
    }

    def __init__(self):
        super(InitializePath, self).__init__(
            outcomes=_outcomes,
            input_keys=[
                "pathId",
                "times",
                "transitionIds",
                "endStateIds",
                "currentSection",
            ],
            output_keys=["transitionId", "endStateId", "currentSection", "duration"],
        )

        self.targetPub = ros_tools.createPublishers(
            "/hpp/target", self.hppTargetPubDict
        )
        self.serviceProxies = ros_tools.createServiceProxies(
            "", InitializePath.serviceProxiesDict
        )
        self.hppclient = HppClient(context="corbaserver")

    def execute(self, userdata):
        userdata.currentSection += 1
        if userdata.currentSection + 1 >= len(userdata.times):
            return _outcomes[0]
        start = userdata.times[userdata.currentSection]
        length = userdata.times[userdata.currentSection + 1] - start

        transitionId = userdata.transitionIds[userdata.currentSection]
        userdata.transitionId = transitionId
        userdata.endStateId = userdata.endStateIds[userdata.currentSection]
        userdata.duration = length

        wait_if_step_by_step("Preparing to read subpath.", 3)

        manip = self.hppclient._manip()
        manip.graph.selectGraph(transitionId[1])
        self.targetPub["read_subpath"].publish(
            ReadSubPath(userdata.pathId, start, length)
        )
        rospy.loginfo("Start reading subpath.")
        return _outcomes[2]


## Execute a sub-path
#
# There are three main steps in execution of a sub-path
# \li pre-action tasks
# \li execution of the path
# \li post-action tasks, typically closing the gripper
#
# Between each step, the execution is paused until a message on topic
#  `/agimus/sot/event/done`.
#  \todo fix scheduling, the current code waits a bit before considering
#       control_norm_changed and step_by_step messages.
#
#  \todo error handling, like object not in gripper when closing it.
class PlayPath(smach.State):
    hppTargetPubDict = {"publish": [Empty, 1]}
    subscribersDict = {
        "agimus": {
            "sot": {
                "event": {
                    "error": [Int32, "_handleEventError"],
                    "done": [Int32, "_handleEventDone"],
                },
                "interrupt": [String, "_handleInterrupt"],
            }
        },
        "hpp": {"target": {"publish_done": [Empty, "_handlePublishDone"]}},
    }
    serviceProxiesDict = {
        "agimus": {
            "sot": {
                "run_pre_action": [PlugSot],
                "plug_sot": [PlugSot],
                "run_post_action": [PlugSot],
                "clear_queues": [std_srvs.srv.Trigger],
                "read_queue": [ReadQueue],
                "stop_reading_queue": [std_srvs.srv.Empty],
            }
        },
        "hpp": {
            "target": {
                "publish_first": [std_srvs.srv.Trigger],
                "get_queue_size": [GetInt],
            }
        },
    }

    def __init__(self):
        super(PlayPath, self).__init__(
            outcomes=_outcomes,
            input_keys=["transitionId", "endStateId", "duration", "queue_initialized"],
            output_keys=["queue_initialized"],
        )

        self.targetPub = ros_tools.createPublishers(
            "/hpp/target", self.hppTargetPubDict
        )
        self.subscribers = ros_tools.createSubscribers(self, "", self.subscribersDict)
        self.serviceProxies = ros_tools.createServiceProxies(
            "", PlayPath.serviceProxiesDict
        )

        self.path_published = False
        self.event_done = False
        self.event_error = None
        self.interruption = None
        self.event_done_count = 0

    def _handleEventError(self, msg):
        self.event_error = msg.data

    def _handleEventDone(self, msg):
        self.event_done_count += 1
        # self.event_done = msg.data
        self.event_done = True
        rospy.loginfo(
            "Received event_done "
            + str(msg.data)
            + " ("
            + str(self.event_done_count)
            + ")"
        )

    def _handleInterrupt(self, msg):
        self.interruption = msg.data
        rospy.loginfo(str(msg.data))
        self.path_published = True

    def _handlePublishDone(self, msg):
        self.path_published = True
        rospy.loginfo("Publishing path done.")

    def _wait_for_event_done(self, rate, msg):
        rospy.loginfo("Wait for event on /agimus/sot/event/done")
        while not self.event_done:
            if self.event_error:
                self.event_done = False
                raise ErrorEvent("ErrorEvent during " + msg)
            if rospy.is_shutdown():
                raise ErrorEvent("Requested rospy shutdown")
            rate.sleep()
        self.event_done_count -= 1
        self.event_done = False

    def execute(self, userdata):
        rate = rospy.Rate(1000)

        try:
            wait_if_step_by_step("Beginning execution.", 3)

            first_published = False
            if not userdata.queue_initialized:
                rsp = self.serviceProxies["hpp"]["target"]["publish_first"]()
                if not rsp.success:
                    raise ErrorEvent(
                        "Could not initialize the queues in SoT: " + rsp.message
                    )
                # TODO make sure the message have been received
                # It *should* be fine to do it with read_queue as the current
                # sot *should* be "keep_posture"
                self.serviceProxies["agimus"]["sot"]["clear_queues"]()
                userdata.queue_initialized = True
                first_published = True
                rospy.loginfo("Queues initialized.")

            # TODO Check that there the current SOT and the future SOT are compatible ?
            self.serviceProxies["agimus"]["sot"]["clear_queues"]()
            status = self.serviceProxies["agimus"]["sot"]["run_pre_action"](
                userdata.transitionId[0], userdata.transitionId[1]
            )
            if status.success:
                rospy.loginfo("Start pre-action")
                if not first_published:
                    rsp = self.serviceProxies["hpp"]["target"]["publish_first"]()
                    if not rsp.success:
                        raise ErrorEvent(rsp.message)
                    self.event_done = False
                    self.serviceProxies["agimus"]["sot"]["read_queue"](
                        delay=1, minQueueSize=1, duration=0
                    )
                    first_published = True
                else:
                    self.event_done = False
                    self.serviceProxies["agimus"]["sot"]["read_queue"](
                        delay=1, minQueueSize=0, duration=0
                    )

                self._wait_for_event_done(rate, "pre-actions")
                wait_if_step_by_step("Pre-action ended.", 2)

            rospy.loginfo("Publishing path")
            self.path_published = False
            self.serviceProxies["agimus"]["sot"]["clear_queues"]()
            queueSize = self.serviceProxies["hpp"]["target"]["get_queue_size"]().data
            self.targetPub["publish"].publish()

            status = self.serviceProxies["agimus"]["sot"]["plug_sot"](
                userdata.transitionId[0], userdata.transitionId[1]
            )
            if not status.success:
                rospy.logerr(status.msg)
                return _outcomes[1]

            # self.control_norm_ok = False
            rospy.loginfo("Read queue (size {})".format(queueSize))
            #  SoT should wait to have a queue larger than 1. This ensures that read_queue won't run into
            #  an infinite loop for a very short path (i.e. one configuration only).
            #  SoT should not wait to have a queue larger than 100
            #  Delay is 1 if the queue is large enough or 10 if it is small.
            # TODO Make maximum queue size and delay parameterizable.
            queueSize = min(max(queueSize, 1), 100)
            delay = 1 if queueSize > 10 else 10
            self.event_done = False
            self.serviceProxies["agimus"]["sot"]["read_queue"](
                delay=delay, minQueueSize=queueSize, duration=userdata.duration
            )

            if self.interruption is not None:
                rospy.logerr(str(self.interruption))
                self.interruption = None
                return _outcomes[2]

            self._wait_for_event_done(rate, "main action")
            while not self.path_published:
                # rospy.logerr("Path publication is not over yet.")
                rate.sleep()
                # TODO stop publishing queues

            wait_if_step_by_step("Action ended.", 2)

            # Run post action if any
            rospy.loginfo("Start post-action")
            self.event_done = False
            status = self.serviceProxies["agimus"]["sot"]["run_post_action"](
                userdata.endStateId[0], userdata.endStateId[1]
            )

            if status.success:
                self._wait_for_event_done(rate, "post-action")
                wait_if_step_by_step("Post-action ended.", 2)

            return _outcomes[0]
        except ErrorEvent as e:
            # TODO interrupt path publication.
            rospy.logerr(str(e))
            return _outcomes[1]


## This class handles user input.
#
#  \li wait for user input
#  \li obtain the waypoints of path from HPP
#  \li duplicated first waypoint. This ensures that SoT will start the motion
#      from the right configuration.
#  \li get the transition for each sub-path and remove *fake transitions*.
#     See comment in the code.
#     \todo I think this should be done in HPP.
#  \li initalize the topics. \n
#     (call services `hpp/target/reset_topics` and `agimus/sot/request_hpp_topics`)
class WaitForInput(smach.State):
    #  Accessed services
    serviceProxiesDict = {
        "agimus": {
            "sot": {
                "request_hpp_topics": [std_srvs.srv.Trigger],
                "plug_sot": [PlugSot],
                "set_base_pose": [SetPose],
            }
        },
        "hpp": {"target": {"reset_topics": [std_srvs.srv.Empty]}},
    }

    def __init__(self):
        super(WaitForInput, self).__init__(
            outcomes=["succeeded", "aborted"],
            input_keys=[],
            output_keys=[
                "pathId",
                "times",
                "transitionIds",
                "endStateIds",
                "currentSection",
                "queue_initialized",
            ],
        )

        rospy.logwarn("Create service WaitForInput")
        self.services = ros_tools.createServiceProxies("", self.serviceProxiesDict)
        self.status_srv = rospy.Service("status", std_srvs.srv.Trigger, self.getStatus)
        self.hppclient = HppClient(context="corbaserver")
        self.ready = False
        self.status = "not started"

    def getStatus(self, empty):
        return self.ready, self.status

    def execute(self, userdata):
        self.status = "waiting"
        self.ready = True
        res = rospy.wait_for_message("start_path", UInt32)
        self.ready = False
        pid = res.data
        self.status = "playing path " + str(pid)
        rospy.loginfo("Requested to start path " + str(pid))
        userdata.pathId = pid
        userdata.queue_initialized = False
        try:
            hpp = self.hppclient._hpp()
            manip = self.hppclient._manip()
            qs, ts = hpp.problem.getWaypoints(pid)
            # Add a first section to force going to init pose.
            # ts: list of transition times. The first one is repeated to make the robot
            # move to the initial configuration before any other motion.
            ts = ts[0:1] + ts
            # tids: list of pair (transitionName, graphName)
            tids = [
                manip.problem.edgeAtParam(pid, (t0 + t1) / 2)
                for t0, t1 in zip(ts[:-1], ts[1:])
            ]
            for i, tid in enumerate(tids):
                manip.graph.selectGraph(tid[1])
                tids[i] = (manip.graph.getName(tid[0]), tid[1])
            # print len(qs), len(ts), len(tids)
            # Remove fake transitions (i.e. when id is the same for two consecutive transitions)
            tts = ts[0:3]
            ttids = tids[0:2]
            tqs = qs[0:2]
            for t, id, q in zip(ts[2:], tids[1:], qs[1:]):
                if ttids[-1] == id:
                    tts[-1] = t
                    tqs[-1] = q
                else:
                    ttids.append(id)
                    tts.append(t)
                    tqs.append(q)

            userdata.times = tuple(tts)
            userdata.transitionIds = tuple(ttids)
            endStateIds = []
            for q, tid in zip(tqs, ttids):
                manip.graph.selectGraph(tid[1])
                nid = manip.graph.getNode(q)
                endStateIds.append((manip.graph.getName(nid), tid[1]))
            userdata.endStateIds = tuple(endStateIds)
            userdata.currentSection = -1
            # TODO reset_topics should not be necessary
            self.services["hpp"]["target"]["reset_topics"]()
            self.services["agimus"]["sot"]["request_hpp_topics"]()
            # Set base pose
            # TODO shouldn't this be done by the estimation node ?
            # moreover, this assumes that HPP has a free-floating base.
            from agimus_hpp.tools import hppPoseToSotTransRPY

            self.services["agimus"]["sot"]["set_base_pose"](
                *hppPoseToSotTransRPY(tqs[0][:7])
            )
            rospy.sleep(0.001)
            # TODO check that qs[0] and the current robot configuration are
            # close
        except Exception as e:
            rospy.logerr("Failed " + str(e))
            return "aborted"
        return "succeeded"


def makeStateMachine():
    # Set default values of parameters
    if not rospy.has_param("step_by_step"):
        rospy.set_param("step_by_step", 0)

    sm = smach.StateMachine(outcomes=_outcomes)

    with sm:
        smach.StateMachine.add(
            "WaitForInput",
            WaitForInput(),
            transitions={"succeeded": "Init", "aborted": "WaitForInput"},
            remapping={
                "pathId": "pathId",
                "times": "times",
                "transitionIds": "transitionIds",
                "endStateIds": "endStateIds",
                "currentSection": "currentSection",
                "queue_initialized": "queue_initialized",
            },
        )
        smach.StateMachine.add(
            "Init",
            InitializePath(),
            transitions={
                "succeeded": "WaitForInput",
                "aborted": "aborted",
                "preempted": "Play",
            },
            remapping={
                "pathId": "pathId",
                "times": "times",
                "transitionId": "transitionId",
                "currentSection": "currentSection",
            },
        )
        smach.StateMachine.add(
            "Play",
            PlayPath(),
            transitions={
                "succeeded": "Init",
                "aborted": "WaitForInput",
                "preempted": "WaitForInput",
            },
            remapping={
                "transitionId": "transitionId",
                "duration": "duration",
                "queue_initialized": "queue_initialized",
            },
        )

    sm.set_initial_state(["WaitForInput"])

    sis = smach_ros.IntrospectionServer("agimus", sm, "/AGIMUS")
    return sm, sis
