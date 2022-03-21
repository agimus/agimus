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
from agimus_sot_msgs.msg import ReadSubPath
from agimus_sot_msgs.srv import PlugSot
from std_msgs.msg import Empty

## State of \c smach finite-state machine
#
#  See method \link agimus.path_execution.initialize_path.InitializePath.execute
#  execute \endlink for details.
class InitializePath(smach.State):
    hppTargetPubDict = {"/hpp/target/read_subpath": [ReadSubPath, 1],
                        "/agimus/status/path_success": [Empty, 1]}
    serviceProxiesDict = {
        "agimus": {
            "sot": {
                "clear_queues": [std_srvs.srv.Trigger],
                "plug_sot": [PlugSot],
            }
        },
        "hpp": {"target": {"publish_first": [std_srvs.srv.Trigger]}},
    }

    def __init__(self, status, hppclient):
        super(InitializePath, self).__init__(
            outcomes=["finished", "move_base", "next"],
            input_keys=[
                "pathId",
                "times",
                "transitionIds",
                "endStateIds",
                "currentSection",
            ],
            output_keys=[ "currentSection",
                "times", "pathId",
                "transitionId", "endStateId", "duration"],
        )
        self.status = status

        self.targetPub = ros_tools.createPublishers(
            "", self.hppTargetPubDict
        )
        self.serviceProxies = ros_tools.createServiceProxies(
            "", InitializePath.serviceProxiesDict
        )
        self.hppclient = hppclient


    ## Initialize the trajectory publisher.
    #
    # If last section of the path has been executed, returns "succeeded",
    # otherwise,
    # \li stores values corresponding to the current path segment in
    # \code
    # userdata.transitionId
    # userdata.endStateId
    # userdata.duration
    # \endcode
    # \li publish path id start time and duration of current segment in
    #     topic "/hpp/target/read_subpath".
    # and returns "preempted".
    #
    # \sa agimus_hpp.trajectory_publisher.HppOutputQueue

    def execute(self, userdata):
        userdata.currentSection += 1
        if userdata.currentSection + 1 >= len(userdata.times):
            status = self.serviceProxies["agimus"]["sot"]["plug_sot"]("", "")
            if not status.success:
                rospy.logerr ("Could not change controller to 'keep_posture': " + status.msg)
            self.targetPub["/agimus/status/path_success"].publish()
            return "finished"

        transitionId = userdata.transitionIds[userdata.currentSection]
        if transitionId[0] == "move_base":
            # go to move base state.
            return "move_base"
        else:
            start = userdata.times[userdata.currentSection]
            length = userdata.times[userdata.currentSection + 1] - start

            userdata.transitionId = transitionId
            userdata.endStateId = userdata.endStateIds[userdata.currentSection]
            userdata.duration = length

            self.status.wait_if_step_by_step("Preparing to read subpath.", 3)

            self.status.set_description("Preparing publication of subpath {}, action {}."
                    .format(userdata.pathId, transitionId[0]))
            manip = self.hppclient._manip()
            manip.graph.selectGraph(transitionId[1])
            self.targetPub["/hpp/target/read_subpath"].publish(
                ReadSubPath(userdata.pathId, start, length)
            )
            rospy.loginfo("Start reading subpath.")
            return "next"
