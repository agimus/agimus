#!/usr/bin/env python
# Copyright 2019 CNRS Airbus SAS
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

## State of \c smach finite-state machine
#
#  See method \link agimus.path_execution.error_state.ErrorState.execute
#  execute \endlink for details.
class ErrorState(smach.State):
    serviceProxiesDict = {
        "agimus": {
            "sot": {
                "clear_queues": [std_srvs.srv.Trigger],
                "plug_sot": [PlugSot],
            }
        },
    }

    def __init__(self, status):
        super(ErrorState, self).__init__(
            outcomes=["finished",],
            input_keys=[],
            output_keys=[],
        )
        self.status = status

        self.serviceProxies = ros_tools.createServiceProxies(
            "", ErrorState.serviceProxiesDict
        )

    ## Handle errors
    # - stop the robot,
    # - clear the queues of references in SoT.
    # \todo stop publication of trajectory by agimus_hpp.
    def execute(self, userdata):
        status = self.serviceProxies["agimus"]["sot"]["plug_sot"]("", "")
        if not status.success:
            rospy.logerr ("Could not change controller to 'keep_posture': " + status.msg)
        self.serviceProxies["agimus"]["sot"]["clear_queues"]()
        return "finished"
