import rospy
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QFrame,
    QGridLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QWidget,
)
from qt_gui.plugin import Plugin
from std_msgs.msg import Empty as EmptyMsg, Bool, Int64, UInt32, String
from std_srvs.srv import Empty as EmptySrv


class PathExecution(Plugin):
    StepByStepParam = "/agimus/step_by_step"
    StepTopic = "/agimus/step"
    PathExecutionTopic = "/agimus/start_path"
    PublishStateService = "/agimus/sot/publish_state"
    EventDoneTopic = "/agimus/sot/event/done"
    EventErrorTopic = "/agimus/sot/event/error"
    StatusDescriptionTopic = "/agimus/status/description"
    StatusWaitStepByStepTopic = "/agimus/status/is_waiting_for_step_by_step"
    StatusRunningTopic = "/agimus/status/running"
    StatusWaitEventDoneTopic = "/agimus/status/is_waiting_for_event_done"

    def __init__(self, context):
        super(PathExecution, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName("PathExecution")

        while not rospy.has_param(PathExecution.StepByStepParam):
            from time import sleep

            rospy.loginfo("Waiting for parameter " + PathExecution.StepByStepParam)
            sleep(0.1)

        self.step_publisher = rospy.Publisher(
            PathExecution.StepTopic, EmptyMsg, queue_size=1
        )
        self.path_execution_publisher = rospy.Publisher(
            PathExecution.PathExecutionTopic, UInt32, queue_size=1
        )
        self.event_done_publisher = rospy.Publisher(
            PathExecution.EventDoneTopic, Int64, queue_size=1
        )
        self.event_error_publisher = rospy.Publisher(
            PathExecution.EventErrorTopic, Int64, queue_size=1
        )

        # Create QWidget
        self._widget = QWidget()
        self._layout = QGridLayout(self._widget)
        self._layout.setColumnStretch(0, 1)
        self._layout.setColumnStretch(3, 1)

        def add_space(row):
            spacer = QFrame()
            spacer.setFrameShape(QFrame.HLine)
            # self._layout.addWidget (spacer, row, 0, 1, 4)
            self._layout.addWidget(spacer, row, 1, 1, 2)

        row = 0

        # Status
        self._layout.addWidget(QLabel("Current status"), row, 1, 1, 2, Qt.AlignCenter)
        row += 1

        self._status_desc = QLabel()
        self._status_desc.setWordWrap(True);
        self._layout.addWidget(self._status_desc, row, 1, 1, 2, Qt.AlignLeft)
        row += 1

        # Spacer
        add_space(row)
        row += 1

        # Step by step
        self._layout.addWidget(QLabel("Step by step"), row, 1, 1, 2, Qt.AlignCenter)
        row += 1

        step_by_step_spin_box = QSpinBox()
        step_by_step_spin_box.setRange(0, 4)
        step_by_step_spin_box.setToolTip(
            """Set the step by step level, from 0 (non-stop) to 4 (stop very often)."""
        )
        step_by_step_spin_box.setValue(rospy.get_param(PathExecution.StepByStepParam))
        # step_by_step_spin_box.connect (self.setStepByStepParam)
        step_by_step_spin_box.valueChanged.connect(
            lambda val: rospy.set_param(PathExecution.StepByStepParam, val)
        )
        self._layout.addWidget(QLabel("Level: "), row, 1, Qt.AlignRight)
        self._layout.addWidget(step_by_step_spin_box, row, 2)
        row += 1

        self._one_step_button = QPushButton("Execute one step")
        self._one_step_button.clicked.connect(lambda x: self.step_publisher.publish())
        self._one_step_button.setEnabled(False)
        self._layout.addWidget(self._one_step_button, row, 2)
        row += 1

        # Spacer
        add_space(row)
        row += 1

        ## Path execution
        self._layout.addWidget(QLabel("Path execution"), row, 1, 1, 2, Qt.AlignCenter)
        row += 1

        self.path_index_spin_box = QSpinBox()
        self.path_index_spin_box.setRange(0, 10000)
        self._execute_path = QPushButton("Execute path")
        self._execute_path.clicked.connect(
            lambda unused: self.path_execution_publisher.publish(
                self.path_index_spin_box.value()
            )
        )
        self._layout.addWidget(self.path_index_spin_box, row, 1)
        self._layout.addWidget(self._execute_path, row, 2)
        row += 1

        # Spacer
        add_space(row)
        row += 1

        ## Tools
        self._layout.addWidget(QLabel("Tools"), row, 1, 1, 2, Qt.AlignCenter)
        row += 1

        publish_state = QPushButton("Request SoT to publish its state")
        publish_state.clicked.connect(lambda u: self.publishState())
        self._layout.addWidget(publish_state, row, 2)
        row += 1

        self._send_event_done = QPushButton("Trigger event done.")
        self._send_event_done.clicked.connect(lambda x: self.event_done_publisher.publish(0))
        self._send_event_done.setEnabled(False)
        self._layout.addWidget(self._send_event_done, row, 2)
        row += 1

        send_event_error = QPushButton("Trigger event error.")
        send_event_error.setStyleSheet("QPushButton { background-color: red }")
        send_event_error.clicked.connect(lambda x: self.event_error_publisher.publish(0))
        self._layout.addWidget(send_event_error, row, 2)
        row += 1

        geom_simu_paused = QPushButton("Pause geometric simu.")
        geom_simu_paused.setCheckable(True)
        geom_simu_paused.setChecked(rospy.get_param("/geometric_simu/paused", False))
        geom_simu_paused.clicked.connect(
            lambda checked: rospy.set_param("/geometric_simu/paused", checked)
        )
        self._layout.addWidget(geom_simu_paused, row, 2)
        row += 1

        reset_tag_poses = QPushButton("Reset tag poses.")
        reset_tag_poses.clicked.connect(self.reset_tag_poses)
        self._layout.addWidget(reset_tag_poses, row, 2)
        row += 1

        space = QWidget()
        space.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._layout.addWidget(space, row, 1, 1, 2)
        row += 1

        # self._layout.addWidget (None, row, 0, 1, 4)

        # Give QObjects reasonable names
        self._widget.setObjectName("SupervisionUi")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.subs = {
                "status/description": rospy.Subscriber (PathExecution.StatusDescriptionTopic,
                    String, lambda msg: self._status_desc.setText(msg.data)),
                "status/is_waiting_for_step_by_step": rospy.Subscriber (PathExecution.StatusWaitStepByStepTopic,
                    Bool, lambda msg: self._one_step_button.setEnabled(msg.data)),
                "status/is_waiting_for_event_done": rospy.Subscriber (PathExecution.StatusWaitEventDoneTopic,
                    Bool, lambda msg: self._send_event_done.setEnabled(msg.data)),
                "status/running": rospy.Subscriber (PathExecution.StatusRunningTopic,
                    Bool, lambda msg: self._execute_path.setEnabled(not msg.data)),
                }

    def publishState(self):
        try:
            rospy.wait_for_service(PathExecution.PublishStateService, 0.5)
            publish_state = rospy.ServiceProxy(
                PathExecution.PublishStateService, EmptySrv
            )
            publish_state()
        except rospy.exceptions.ROSException as e:
            QMessageBox.warning(self, "Service unreachable.", str(e))

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def reset_tag_poses(self):
        try:
            from std_srvs.srv import Trigger
            reset = rospy.ServiceProxy ("/vision/reset_tag_poses", Trigger)
            reset()
        except rospy.ServiceException:
            pass

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog
