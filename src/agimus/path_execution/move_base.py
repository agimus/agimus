#!/usr/bin/env python
# Copyright 2020 CNRS Airbus SAS
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
from agimus_hpp import ros_tools
from agimus_sot_msgs.srv import GetBasePoseAtParam

from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def _PoseToSE3(pose):
    from pinocchio import XYZQUATToSE3
    return XYZQUATToSE3([ pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w ])

def _demangle_status(s):
    from actionlib import GoalStatus
    if GoalStatus.ABORTED == s: return "ABORTED"
    if GoalStatus.ACTIVE == s: return "ACTIVE"
    if GoalStatus.SUCCEEDED == s: return "SUCCEEDED"
    if GoalStatus.PENDING == s: return "PENDING"
    if GoalStatus.LOST == s: return "LOST"
    if GoalStatus.PREEMPTED == s: return "PREEMPTED"
    if GoalStatus.RECALLED == s: return "RECALLED"
    if GoalStatus.RECALLING == s: return "RECALLING"
    if GoalStatus.REJECTED == s: return "REJECTED"
    return s

class _Feedback:
    def __init__(self,
            axclient,
            get_pose_func,
            start,
            end,
            step=0.1,
            distance_thr=0.3
            ):
        self.axclient = axclient
        self.get_pose = get_pose_func
        self.t = start
        self.step = step
        self.end = end
        self.distance_thr = distance_thr

        self.path_completed = False
        self.abort = False
        self._send_next_goal()

    def _send_goal(self,t):
        rospy.loginfo("Feedback: sending goal " + str(t))
        rsp = self.get_pose(t)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = rsp.pose
        self._started=False
        self._goal = _PoseToSE3(rsp.pose)
        self.axclient.send_goal(goal,
                done_cb=self.done,
                active_cb=self.active,
                feedback_cb=self.feedback)

    def _send_next_goal(self):
        if self.step == 0: return
        self.t += self.step
        if self.t >= self.end:
            self.t = self.end
            self.step = 0
        self._send_goal(self.t)

    def active (self):
        self._started = True
        self._next_goal_sent = False

    def feedback (self, msg):
        if not self._started: return
        # compute distance to goal
        cur = _PoseToSE3(msg.base_position.pose)
        import pinocchio, numpy as np
        w = pinocchio.log6(cur.inverse()*self._goal)
        if np.linalg.norm(w) < self.distance_thr:
            self._next_goal_sent = True
            self._send_next_goal()

    def done (self, state, msg):
        if not self._started: return
        rospy.loginfo("Feedback.done: " + _demangle_status(state))
        #if status != GoalStatus.SUCCEEDED:
        #        self.abort = true
        #        rospy.logerr("Could not achieve move_base goal : " + _demangle_status(status))
        #        return "preempted"
        if self.step == 0:
            self.path_completed = True
        elif not self._next_goal_sent:
            #TODO do I need this ?
            self._send_next_goal()

## Mobile base motion
class MoveBase(smach.State):
    serviceProxiesDict = {
        "hpp": {
            "target": {
                "get_base_pose_at_param": [GetBasePoseAtParam],
                }
        },
    }

    def __init__(self, status):
        super(MoveBase, self).__init__(
            outcomes=["succeeded", "preempted"],
            input_keys=["pathId", "currentSection", "times", ],
        )
        self.status = status

        self.serviceProxies = ros_tools.createServiceProxies(
            "", MoveBase.serviceProxiesDict
        )

        self.axclient = SimpleActionClient("/move_base", MoveBaseAction)

    def execute(self, userdata):
        pathId = userdata.pathId
        start = userdata.times[userdata.currentSection]
        end = userdata.times[userdata.currentSection + 1]

        client = _Feedback(self.axclient,
                lambda t: self.serviceProxies["hpp"]["target"]["get_base_pose_at_param"](pathId, t),
                start, end,
                step=0.1)

        rate = rospy.Rate(10)
        while not client.path_completed:
            if client.abort: return "preempted"
            rate.sleep()
        return "succeeded"
