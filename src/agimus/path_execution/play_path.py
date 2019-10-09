#!/usr/bin/env python
# Copyright 2018 CNRS Airbus SAS
# Author: Joseph Mirabel

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import smach
import smach_ros
import std_srvs.srv
from agimus_hpp import ros_tools
from agimus_hpp.client import HppClient
from agimus_sot_msgs.srv import GetInt, PlugSot, ReadQueue
from std_msgs.msg import Empty, Int32, String, UInt32
from initialize_path import InitializePath, wait_if_step_by_step
from wait_for_input import WaitForInput
from outcome import _outcomes


class ErrorEvent(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


## State of \c smach finite-state machine
#
#  See method \link agimus.path_execution._play_path.PlayPath.execute
#  execute \endlink for details.
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
                raise ErrorEvent("ErrorEvent during {}: {}".format(msg, self.event_error))
            if rospy.is_shutdown():
                raise ErrorEvent("Requested rospy shutdown")
            rate.sleep()
        self.event_done_count -= 1
        self.event_done = False

    ## Execute a sub-path
    #
    # There are three main steps in execution of a sub-path
    # \li pre-action tasks
    # \li execution of the path, publish in topic "/hpp/target/publish"
    # \li post-action tasks, typically closing the gripper
    #
    # Between each step, the execution is paused until a message on topic
    # `/agimus/sot/event/done`.
    # \todo fix scheduling, the current code waits a bit before considering
    #       control_norm_changed and step_by_step messages.
    #
    # \todo error handling, like object not in gripper when closing it.
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
                # TODO make sure the message has been received
                # It *should* be fine to do it with read_queue as the current
                # sot *should* be "keep_posture"
                self.serviceProxies["agimus"]["sot"]["clear_queues"]()
                userdata.queue_initialized = True
                first_published = True
                rospy.loginfo("Queues initialized.")

            # TODO Check that the current SOT and the future SOT are compatible ?
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
            # SoT should wait to have a queue larger than 1. This ensures that read_queue won't run into
            # an infinite loop for a very short path (i.e. one configuration only).
            # SoT should not wait to have a queue larger than 100
            # Delay is 1 if the queue is large enough or 10 if it is small.
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
