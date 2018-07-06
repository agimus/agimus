from python_qt_binding.QtWidgets import QGridLayout, QWidget, QLabel, QSpinBox, QPushButton, QFrame, QMessageBox
from python_qt_binding.QtCore import Qt
from qt_gui.plugin import Plugin

import rospy
from std_msgs.msg import Empty as EmptyMsg, UInt32
from std_srvs.srv import Empty as EmptySrv

import os
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class PathExecution(Plugin):
    StepByStepParam = "step_by_step"
    StepTopic = "step"
    PathExecutionTopic = "start_path"
    PublishStateService = "/agimus/sot/publish_state"

    def __init__(self, context):
        super(PathExecution, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('PathExecution')

        while not rospy.has_param(PathExecution.StepByStepParam):
            rospy.sleep(0.1)

        self.step_publisher = rospy.Publisher (PathExecution.StepTopic, EmptyMsg, queue_size=1)
        self.path_execution_publisher = rospy.Publisher (PathExecution.PathExecutionTopic, UInt32, queue_size=1)

        # Create QWidget
        self._widget = QWidget()
        self._layout = QGridLayout (self._widget)
        self._layout.setColumnStretch (0,1)
        self._layout.setColumnStretch (3,1)

        def add_space (row):
            spacer = QFrame()
            spacer.setFrameShape (QFrame.HLine)
            #self._layout.addWidget (spacer, row, 0, 1, 4)
            self._layout.addWidget (spacer, row, 1, 1, 2)

        row = 0

        # Step by step
        self._layout.addWidget (QLabel("Step by step"), row, 1, 1, 2, Qt.AlignCenter)
        row+=1

        step_by_step_spin_box = QSpinBox ()
        step_by_step_spin_box.setRange(0,4)
        step_by_step_spin_box.setToolTip ("""Set the step by step level, from 0 (non-stop) to 4 (stop very often).""")
        step_by_step_spin_box.setValue(rospy.get_param(PathExecution.StepByStepParam))
        # step_by_step_spin_box.connect (self.setStepByStepParam)
        step_by_step_spin_box.valueChanged.connect (lambda val: rospy.set_param(PathExecution.StepByStepParam, val))
        self._layout.addWidget (QLabel("Level: "), row, 1, Qt.AlignRight)
        self._layout.addWidget (step_by_step_spin_box , row, 2)
        row+=1

        self._one_step_button = QPushButton ("Execute one step")
        self._one_step_button.clicked.connect (lambda x: self.step_publisher.publish())
        self._layout.addWidget (self._one_step_button , row, 2)
        row+=1

        # Spacer
        add_space (row)
        row+=1

        ## Path execution
        self._layout.addWidget (QLabel("Path execution"), row, 1, 1, 2, Qt.AlignCenter)
        row+=1

        self.path_index_spin_box = QSpinBox ()
        # step_by_step_spin_box.setRange(0,100000)
        execute_path = QPushButton ("Execute path")
        execute_path.clicked.connect (lambda unused: self.path_execution_publisher.publish(self.path_index_spin_box.value()))
        self._layout.addWidget (self.path_index_spin_box , row, 1)
        self._layout.addWidget (execute_path, row, 2)
        row+=1

        # Spacer
        add_space (row)
        row+=1

        ## Tools
        self._layout.addWidget (QLabel("Tools"), row, 1, 1, 2, Qt.AlignCenter)
        row+=1

        publish_state = QPushButton ("Request SoT to publish its state")
        publish_state.clicked.connect (lambda u: self.publishState())
        self._layout.addWidget (publish_state, row, 2)
        row+=1

        #self._layout.addWidget (None, row, 0, 1, 4)

        # Give QObjects reasonable names
        self._widget.setObjectName('SupervisionUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def publishState (self):
        try:
            rospy.wait_for_service (PathExecution.PublishStateService, 0.5)
            publish_state = rospy.ServiceProxy (PathExecution.PublishStateService, EmptySrv)
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

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
