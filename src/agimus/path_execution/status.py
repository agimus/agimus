import rospy

## Define the status of the path execution FSM
class Status(object):
    def __init__ (self):
        from std_msgs.msg import Bool, String
        ## A human readable string which tells what the FSM is doing.
        self._description = rospy.Publisher("/agimus/status/description", String, queue_size=1, latch=True)
        ## A boolean which is True if the FSM is waiting for a message on step_by_step topic.
        self._step_by_step = rospy.Publisher("/agimus/status/is_waiting_for_step_by_step", Bool, queue_size=1, latch=True)
        ## A boolean which is True if a path is being executed.
        self._running = rospy.Publisher("/agimus/status/running", Bool, queue_size=1, latch=True)

    def set_description(self, msg):
        self._description.publish (msg)

    ## Waits if the step by step level is lower than level
    #
    # The step by step level is stored in ROS param `step_by_step`.
    # \param msg human readable message.
    # \param level 0 means *always wait*.
    # \param time to wait **after** the message is received.
    #
    # \todo It should be possible to handle errors while waiting for user input.
    def wait_if_step_by_step(self, msg, level, time=0.1):
        from std_msgs.msg import Empty
        l = rospy.get_param("step_by_step", 0)
        if type(l) == bool:
            l = 10 if l else 0
        if level < l:
            rospy.loginfo(
                "{} Wait for message on {}/step.".format(msg, rospy.get_namespace())
            )
            # rospy.wait_for_message (rospy.get_namespace() + "/step", Empty)
            self._step_by_step.publish(True)
            rospy.wait_for_message("step", Empty)
            self._step_by_step.publish(False)
            rospy.sleep(time)

    def set_running(self, msg):
        self._running.publish (msg)
