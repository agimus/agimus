#!/usr/bin/env python
from agimus.path_execution import makeStateMachine
import rospy, smach

def run():
    rospy.init_node("agimus")
    sm, sis = makeStateMachine()

    sis.start()
    outcome = sm.execute()
    sis.stop()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
    except smach.InvalidUserCodeError as e:
        if "ROSInterruptException" in str(e):
            raise Exception ("I was able to catch the following exception as a ROSInterruptException\n" + str(e))
        else:
            raise e
