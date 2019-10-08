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
import std_srvs.srv
from agimus_hpp import ros_tools
from agimus_hpp.client import HppClient
from agimus_sot_msgs.msg import ReadSubPath
from outcome import _outcomes

## Waits if the step by step level is lower than level
#
# The step by step level is stored in ROS param `step_by_step`.
# \param msg human readable message.
# \param level 0 means *always wait*.
# \param time to wait **after** the message is received.
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



##_Initialize the trajectory publisher.
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


