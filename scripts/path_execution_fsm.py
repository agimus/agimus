#!/usr/bin/env python
from agimus.path_execution import makeStateMachine
import rospy

rospy.init_node("agimus")
sm, sis = makeStateMachine()

sis.start()
outcome = sm.execute()
sis.stop()
