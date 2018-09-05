import smach, smach_ros, rospy
from std_msgs.msg import UInt32, Empty, String, Float64
from agimus_sot_msgs.msg import *
from agimus_sot_msgs.srv import *
from agimus_hpp.client import HppClient
import std_srvs.srv
from agimus_hpp import ros_tools

_outcomes = ["succeeded", "aborted", "preempted"]

def wait_if_step_by_step(msg, level, time=0.1):
    l = rospy.get_param ("step_by_step", 0)
    if type(l)==bool: l = 10 if l else 0
    if level < l:
        rospy.loginfo("{} Wait for message on {}/step.".format(msg, rospy.get_namespace()))
        # rospy.wait_for_message (rospy.get_namespace() + "/step", Empty)
        rospy.wait_for_message ("step", Empty)
        rospy.sleep(time)

class InitializePath(smach.State):
    hppTargetPubDict = {
            "read_path": [ UInt32, 1 ],
            "read_subpath": [ ReadSubPath, 1 ],
            }
    hppTargetSrvDict = {
            "reset_topics": [ std_srvs.srv.Empty, ],
            "add_center_of_mass": [ SetString, ],
            "add_operational_frame": [ SetString, ],
            "add_center_of_mass_velocity": [ SetString, ],
            "add_operational_frame_velocity": [ SetString, ],
            }

    def __init__(self):
        super(InitializePath, self).__init__(
                outcomes = _outcomes,
                input_keys = [ "pathId", "times", "transitionIds", "endStateIds", "currentSection" ],
                output_keys = [ "transitionId", "endStateId", "currentSection" ],
                )

        self.targetSrv = ros_tools.createServiceProxies ("/hpp/target", self.hppTargetSrvDict)
        self.targetPub = ros_tools.createPublishers ("/hpp/target", self.hppTargetPubDict)
        self.hppclient = HppClient (False)

    def execute (self, userdata):
        userdata.currentSection += 1
        if userdata.currentSection + 1 >= len(userdata.times):
            return _outcomes[0]
        start = userdata.times[userdata.currentSection]
        length = userdata.times[userdata.currentSection + 1] - start

        transitionId = userdata.transitionIds[userdata.currentSection]
        userdata.transitionId = transitionId
        userdata.endStateId   = userdata.endStateIds[userdata.currentSection]

        wait_if_step_by_step ("Preparing to read subpath.", 3)

        self.hppclient.manip.graph.selectGraph (transitionId[1])
        self.targetPub["read_subpath"].publish (ReadSubPath (userdata.pathId, start, length))
        rospy.loginfo("Start reading subpath.")
        return _outcomes[2]

class PlayPath (smach.State):
    hppTargetPubDict = {
            "publish": [ Empty, 1 ],
            }
    subscribersDict = {
            "agimus" : {
                "sot": {
                    "error": [ String, "handleError" ],
                    "interrupt": [ String, "handleInterrupt" ],
                    "control_norm_changed": [ Float64, "handleControlNormChanged" ],
                    },
                },
            "hpp" : {
                "target": {
                    "publish_done": [ Empty, "handleFinished" ]
                    }
                }
            }
    serviceProxiesDict = {
            "agimus" : {
                "sot": {
                    'run_pre_action': [ PlugSot, ],
                    'plug_sot': [ PlugSot, ],
                    'run_post_action': [ PlugSot, ],
                    'clear_queues': [ std_srvs.srv.Trigger, ],
                    'read_queue': [ SetInt, ],
                    'stop_reading_queue': [ std_srvs.srv.Empty, ],
                    },
                },
            'hpp': {
                'target': {
                    'publish_first': [ std_srvs.srv.Empty, ],
                    }
                }
            }

    def __init__(self):
        super(PlayPath, self).__init__(
                outcomes = _outcomes,
                input_keys = [ "transitionId", "endStateId", ],
                output_keys = [ ])

        self.targetPub = ros_tools.createPublishers ("/hpp/target", self.hppTargetPubDict)
        self.subscribers = ros_tools.createSubscribers (self, "", self.subscribersDict)
        self.serviceProxies = ros_tools.createServiceProxies ("", PlayPath.serviceProxiesDict)

        self.done = False
        self.error = None
        self.interruption = None
        self.control_norm_ok = True

    def handleError (self, msg):
        self.error = msg.data

    def handleInterrupt (self, msg):
        self.interruption = msg.data
        rospy.loginfo(str(msg.data))
        self.done = True

    def handleFinished (self, msg):
        self.done = True

    def handleControlNormChanged (self, msg):
        self.control_norm_ok = msg.data < 1e-2

    def _wait_for_control_norm_changed (self, rate, time_before_control_norm_changed):
        # rospy.sleep(time_before_control_norm_changed)
        try:
            rospy.wait_for_message("/sot_hpp/control_norm_changed", Float64, time_before_control_norm_changed)
        except rospy.ROSException:
            pass
        rospy.loginfo("Wait for event on /sot_hpp/control_norm_changed")
        while not self.control_norm_ok:
            rate.sleep()

    def execute(self, userdata):
        rate = rospy.Rate (1000)
        time_before_control_norm_changed = 1.

        wait_if_step_by_step ("Beginning execution.", 3)

        # TODO Check that there the current SOT and the future SOT are compatible ?
        self.serviceProxies['agimus']['sot']['clear_queues']()
        status = self.serviceProxies['agimus']['sot']['run_pre_action'](userdata.transitionId[0], userdata.transitionId[1])
        if status.success:
            rospy.loginfo("Run pre-action")
            self.serviceProxies["hpp"]["target"]["publish_first"]()
            self.serviceProxies['agimus']['sot']['read_queue'](10)

            self._wait_for_control_norm_changed (rate, time_before_control_norm_changed)
            wait_if_step_by_step ("Pre-action ended.", 2)

        rospy.loginfo("Publishing path")
        self.done = False
        self.serviceProxies['agimus']['sot']['clear_queues']()
        self.targetPub["publish"].publish()

        status = self.serviceProxies['agimus']['sot']['plug_sot'](userdata.transitionId[0], userdata.transitionId[1])
        if not status.success:
            rospy.logerr(status.msg)
            return _outcomes[1]

        # self.control_norm_ok = False
        rospy.loginfo("Read queue")
        self.serviceProxies['agimus']['sot']['read_queue'](10)
        # t = rospy.Time.now()
        # Wait for errors or publish done
        while not self.done:
            if self.error is not None:
                # TODO handle error
                rospy.logerr(str(self.error))
                self.error = None
                return _outcomes[1]
            rate.sleep()
        rospy.loginfo("Publishing path done.")
        if self.interruption is not None:
            rospy.logerr(str(self.interruption))
            self.interruption = None
            return _outcomes[2]

        # Sometimes, the function triggering /sot_hpp/control_norm_changed is a little slow to update
        # and the movement is skipped
        # This won't be a problem when we have a better mean of detecting the end of a movement
        # t = rospy.Time.now() - t
        # self._wait_for_control_norm_changed (rate, max(0,time_before_control_norm_changed-t.to_sec()))
        self._wait_for_control_norm_changed (rate, time_before_control_norm_changed)
        # self.serviceProxies['sot']['stop_reading_queue']()

        wait_if_step_by_step ("Action ended.", 2)

        # Run post action if any
        rospy.loginfo("Run post-action")
        status = self.serviceProxies['agimus']['sot']['run_post_action'](userdata.endStateId[0], userdata.endStateId[1])

        if status.success:
            self._wait_for_control_norm_changed (rate, time_before_control_norm_changed)
            wait_if_step_by_step ("Post-action ended.", 2)

        return _outcomes[0]

class WaitForInput(smach.State):
    serviceProxiesDict = {
            "agimus" : {
                "sot": {
                    'request_hpp_topics': [ std_srvs.srv.Trigger, ],
                    'plug_sot': [ PlugSot, ]
                    },
                },
            'hpp': {
                'target': {
                    "reset_topics": [ std_srvs.srv.Empty, ],
                    }
                }
            }

    def __init__(self):
        super(WaitForInput, self).__init__(
                outcomes = [ "succeeded", "aborted" ],
                input_keys = [ ],
                output_keys = [ "pathId", "times", "transitionIds", "endStateIds", "currentSection" ],
                )

        rospy.logwarn("Create service WaitForInput")
        self.services = ros_tools.createServiceProxies ("", self.serviceProxiesDict)
        self.hppclient = HppClient (False)

    def execute (self, userdata):
        status = self.services["agimus"]['sot']['plug_sot']("", "")
        res = rospy.wait_for_message ("start_path", UInt32)
        pid = res.data
        rospy.loginfo("Requested to start path " + str(pid))
        userdata.pathId = pid
        try:
            hpp = self.hppclient._hpp()
            manip = self.hppclient._manip()
            qs, ts = hpp.problem.getWaypoints(pid)
            # Add a first section to force going to init pose.
            # ts: list of transition times. The first one is repeated to make the robot
            # move to the initial configuration before any other motion.
            ts = ts[0:1] + ts
            # tids: list of pair (transitionName, graphName)
            tids = [ manip.problem.edgeAtParam(pid, (t0 + t1) / 2) for t0,t1 in zip(ts[:-1], ts[1:]) ]
            for i, tid in enumerate(tids):
                manip.graph.selectGraph (tid[1])
                tids[i] = (manip.graph.getName (tid[0]), tid[1])
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
                    ttids.append (id)
                    tts.append(t)
                    tqs.append(q)

            userdata.times = tuple(tts)
            userdata.transitionIds = tuple(ttids)
            endStateIds = []
            for q, tid in zip(tqs, ttids):
                manip.graph.selectGraph (tid[1])
                nid = manip.graph.getNode(q)
                endStateIds.append ( (manip.graph.getName (nid), tid[1]) )
            userdata.endStateIds = tuple (endStateIds)
            userdata.currentSection = -1
            # TODO this should not be necessary
            self.services['hpp']['target']['reset_topics']()
            self.services["agimus"]['sot']['request_hpp_topics']()
            # TODO check that qs[0] and the current robot configuration are
            # close
        except Exception, e:
            rospy.logerr("Failed " + str(e))
            return "aborted"
        return "succeeded"

def makeStateMachine():
    # Set default values of parameters
    if not rospy.has_param ("step_by_step"):
        rospy.set_param ("step_by_step", 0)

    sm = smach.StateMachine (outcomes = _outcomes)

    with sm:
        smach.StateMachine.add ('WaitForInput', WaitForInput(),
                transitions = {
                    "succeeded": 'Init',
                    "aborted": "WaitForInput" },
                remapping = {
                    "pathId": "pathId",
                    "times": "times",
                    "transitionIds": "transitionIds",
                    "endStateIds": "endStateIds",
                    "currentSection": "currentSection",
                    })
        smach.StateMachine.add ('Init', InitializePath(),
                transitions = {
                    "succeeded": "WaitForInput",
                    "aborted": "aborted",
                    "preempted": 'Play'},
                remapping = {
                    "pathId": "pathId",
                    "times": "times",
                    "transitionId": "transitionId",
                    "currentSection": "currentSection",
                    })
        smach.StateMachine.add ('Play', PlayPath(),
                transitions = {
                    "succeeded": 'Init',
                    "aborted": "WaitForInput",
                    "preempted": "WaitForInput"},
                remapping = {
                    "transitionId": "transitionId",
                    })

    sm.set_initial_state(["WaitForInput"])

    sis = smach_ros.IntrospectionServer('agimus', sm, '/AGIMUS')
    return sm, sis
