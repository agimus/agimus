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

## This class handles user input.
#
# \li wait for user input
# \li obtain the waypoints of path from HPP
# \li duplicated first waypoint. This ensures that SoT will start the motion
#     from the right configuration.
# \li get the transition for each sub-path and remove *fake transitions*.
#     See comment in the code.
#     \todo I think this should be done in HPP.
# \li initalize the topics. \n
#     (call services `hpp/target/reset_topics` and `agimus/sot/request_hpp_topics`)

import rospy
import smach
import std_srvs.srv
from agimus_hpp import ros_tools
from agimus_sot_msgs.srv import SetPose, GetJointNames, SetJointNames
from std_msgs.msg import UInt32

## State of \c smach finite-state machine
#
#  See method \link agimus.path_execution.wait_for_input.WaitForInput.execute
#  execute \endlink for details.
class WaitForInput(smach.State):
    # _Accessed services
    serviceProxiesDict = {
        "agimus": {
            "sot": {
                "request_hpp_topics": [std_srvs.srv.Trigger],
                "set_base_pose": [SetPose],
                "get_joint_names": [GetJointNames],
            }
        },
        "hpp": {
            "target": {
                "reset_topics": [std_srvs.srv.Empty],
                'set_joint_names': [SetJointNames]
            }
        }
    }

    def __init__(self, status, hppclient):
        super(WaitForInput, self).__init__(
            outcomes=["start_path", "interrupted", "failed_to_start"],
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
        self.status = status

        rospy.logwarn("Create service WaitForInput")
        self.services = ros_tools.createServiceProxies("", self.serviceProxiesDict)
        self.hppclient = hppclient
        self.ready = False

    def connectToHPP(self):
        success, msg = self.hppclient.tryConnect()
        if not success:
            rospy.logerr(msg)
        return success

    ## Wait for message "start_path"
    #
    #  Message "start_path" contains the path id to be executed. Once received,
    #  \li a client to \c hppcorbaserver retrieves the waypoints and times of
    #      the path to be executed, times are stored in
    #      \code userdata.times\endcode.
    #  \li a list of pairs [\c transitionName, \c graphName] is built
    #      and stored in \code userdata.transitionIds\endcode.
    #  \li consecutive identical transitions are replaced by one single
    #      transition,
    #  \li a list of final states of the transitions is built and stored in
    #      \code userdata.endStateIds\endcode.
    def execute(self, userdata):
        self.status.set_description("Waiting for path to execute.")
        self.status.set_running(False)
        self.ready = True
        try:
            res = rospy.wait_for_message("start_path", UInt32)
        except rospy.ROSInterruptException as e:
            return "interrupted"

        # Reconnect to HPP
        if not self.connectToHPP():
            self.status.set_description("Failed to connect to HPP.")
            return "failed_to_start"

        # Set the joint list in HPP.
        self.updateJointList()

        self.ready = False
        pid = res.data
        self.status.set_description("Playing path {}.".format(pid))
        self.status.set_running(True)
        rospy.loginfo("Requested to start path " + str(pid))
        userdata.pathId = pid
        userdata.queue_initialized = False
        try:
            hpp = self.hppclient._hpp()
            manip = self.hppclient._manip()
            qs, ts = hpp.problem.getWaypoints(pid)

            # Get config to detect inconsistency between start configuration and current config
            from hpp.corbaserver.manipulation import Robot
            from sensor_msgs.msg import JointState

            robot2 = Robot(load=False)
            robotParam = rospy.get_param("/demo")["robots"]
            robotPrefix = list(robotParam.keys())[0] +"/"

            q_current = qs[0][:]
            # Acquire robot state
            msg = rospy.wait_for_message("/joint_states", JointState)
            for ni, qi in zip(msg.name, msg.position):
                jni = robotPrefix + ni
                if robot2.getJointConfigSize(jni) != 1:
                    continue
                try:
                    rk = robot2.rankInConfiguration[jni]
                except KeyError:
                    continue
                assert robot2.getJointConfigSize(jni) == 1
                q_current[rk] = qi

            for qci,qsi in zip(q_current,qs[0]):
                if abs(qci - qsi) > 8*10**-2:
                    rospy.logerr(f"Agimus : Start configuration of path doesn't match with the current configuration : start config :{qs[0]}, current config:{q_current}")
                    return "failed_to_start"
            # End of consistency check

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
            jointNames = hpp.robot.getJointNames()
            if jointNames[0].endswith("root_joint"):
                basePose, = hpp.robot.getJointsPosition(tqs[0], jointNames[:1])
                self.services["agimus"]["sot"]["set_base_pose"](*hppPoseToSotTransRPY(basePose))
            rospy.sleep(0.001)
            # TODO check that qs[0] and the current robot configuration are
            # close
        except rospy.ROSInterruptException as e:
            return "interrupted"
        except Exception as e:
            rospy.logerr("Failed " + str(e))
            return "failed_to_start"
        return "start_path"

    def updateJointList(self):
        jointNames = self.services["agimus"]["sot"]["get_joint_names"]()
        ans = self.services["hpp"]["target"]["set_joint_names"](jointNames.names)
        if not ans.success:
            rospy.logerr("Could not set the joint list of hpp_ros_interface node")

