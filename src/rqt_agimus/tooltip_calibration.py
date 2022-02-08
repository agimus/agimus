from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QWidget,
)
from qt_gui.plugin import Plugin
import QtGui

import rospy
from dynamic_graph_bridge_msgs.srv import RunCommand
from agimus_sot.srdf_parser import parse_srdf

##
#  Qt Widget to change position of gripper in joint frame
#
#  Todo: make names of handle and gripper parameterizable.

class TooltipCalibration(Plugin):
    _verbose = True
    runCommandService="/run_command"
    initCommands = [
        "import numpy as np",
        "from dynamic_graph.sot.core.feature_pose import FeaturePose",
        "from pinocchio import SE3",
        "from dynamic_graph import get_entity_list",
        "tc_entities = get_entity_list()",
        "tc_f = FeaturePose('pregrasp___ur10e/gripper___part/handle_0_feature')",
        "print(tc_entities)"]
    gripper = "ur10e/gripper"

    def __init__(self, context):
        super(TooltipCalibration, self).__init__(context)
        self.setObjectName("Tooltip calibration")
        # Wait for communication with Stack of Tasks
        #rospy.wait_for_service(self.runCommandService)
        # Create QWidget
        self._widget = QWidget()
        layout = QGridLayout()
        self._widget.setLayout(layout)
        self._translation = [0,0,0]
        row = 0
        label = QLabel("gripper:")
        gripperId = QSpinBox()
        gripperId.setRange(0, 0)
        gripperId.valueChanged.connect(self.gripperIdChanged)
        self.gripperName = QLabel("")
        layout.addWidget(label, row, 0)
        layout.addWidget(self.gripperName, row, 1)
        layout.addWidget(gripperId, row, 2)

        row += 1
        label = QLabel("handle:")
        handleId = QSpinBox()
        handleId.setRange(0, 0)
        handleId.valueChanged.connect(self.handleIdChanged)
        self.handleName = QLabel("")
        layout.addWidget(label, row, 0)
        layout.addWidget(self.handleName, row, 1)
        layout.addWidget(handleId, row, 2)

        row += 1
        xSlider = QSlider(Qt.Horizontal)
        xSlider.setMinimum(-100)
        xSlider.setMaximum(100)
        xSlider.setValue(0)
        xSlider.valueChanged.connect(self.xChanged)
        label = QLabel("x")
        #label.setFont(QtGui.QFont("Timesroman", 12))
        self.xValue = QLabel("0.000")

        layout.addWidget(label, row, 0)
        layout.addWidget(xSlider, row, 1)
        layout.addWidget(self.xValue, row, 2)

        row+=1

        ySlider = QSlider(Qt.Horizontal)
        ySlider.setMinimum(-100)
        ySlider.setMaximum(100)
        ySlider.setValue(0)
        ySlider.valueChanged.connect(self.yChanged)
        label = QLabel("y")
        #label.setFont(QtGui.QFont("Timesroman", 12))
        self.yValue = QLabel("0.000")

        layout.addWidget(label, row, 0)
        layout.addWidget(ySlider, row, 1)
        layout.addWidget(self.yValue, row, 2)

        row+=1
        zSlider = QSlider(Qt.Horizontal)
        zSlider.setMinimum(-100)
        zSlider.setMaximum(100)
        zSlider.setValue(0)
        zSlider.valueChanged.connect(self.zChanged)
        label = QLabel("z")
        #label.setFont(QtGui.QFont("Timesroman", 12))
        self.zValue = QLabel("0.000")

        layout.addWidget(label, row, 0)
        layout.addWidget(zSlider, row, 1)
        layout.addWidget(self.zValue, row, 2)

        row+=1
        # Add widget to the user interface
        context.add_widget(self._widget)
        # Parse srdf files to get available grippers and handles
        self.parseSrdf()
        handleId.setRange(0, len(self.handles)-1)
        if len(self.handles)>0:
            self.handleName.setText(self.handles[0])
        gripperId.setRange(0, len(self.grippers)-1)
        if len(self.grippers)>0:
            self.gripperName.setText(self.grippers[0])
        # Wait for service dynamic_graph_bridge/run_command
        rospy.wait_for_service(self.runCommandService)
        self._runCommand = rospy.ServiceProxy(self.runCommandService,
                                              RunCommand)
        self.initializeSot()

    def xChanged(self, val):
        self.xValue.setText("{:.3f}".format(1e-3*val))
        self._translation[0] = 1e-3*val
        self.setSignalValue()

    def yChanged(self, val):
        self.yValue.setText("{:.3f}".format(1e-3*val))
        self._translation[1] = 1e-3*val
        self.setSignalValue()

    def zChanged(self, val):
        self.zValue.setText("{:.3f}".format(1e-3*val))
        self._translation[2] = 1e-3*val
        self.setSignalValue()

    def handleIdChanged(self, val):
        self.handle = self.handles[val]
        self.handleName.setText(self.handle)
        self.updateFeatureName()
        self.runCommand("tc_f = FeaturePose('{}')".format(self._featureName))

    def gripperIdChanged(self, val):
        self.gripper = self.grippers[val]
        self.gripperName.setText(self.gripper)
        self.updateFeatureName()
        self.runCommand("tc_f = FeaturePose('{}')".format(self._featureName))

    def updateFeatureName(self):
        self._featureName = 'pregrasp___{}___{}_feature'.format\
            (self.gripper, self.handle)
        if not self._featureName in self.entities:
            rospy.logerr("{} is not an existing entity".format\
                         (self._featureName))


    ##
    #  Communication with the Stack of Tasks
    def _isNotError (self, runCommandAnswer):
        if len(runCommandAnswer.standarderror) != 0:
            return False, runCommandAnswer.standarderror
        return True, ""

    def runCommand (self, cmd):
        if self._verbose:
            rospy.loginfo(">> " + cmd)
        answer = self._runCommand (cmd)
        if self._verbose and len(answer.standardoutput) > 0:
            rospy.loginfo(answer.standardoutput)
        if len(answer.standardoutput) > 0:
            rospy.loginfo (answer.standardoutput)
        if len(answer.standarderror) > 0:
            rospy.logerr (answer.standarderror)
        return answer

    def initializeSot(self):
        for c in self.initCommands:
            answer = self.runCommand(c)
            success, msg = self._isNotError(answer)
            if not success:
                return
        self.entities = eval(answer.standardoutput)

    def setSignalValue(self):
        self.runCommand("tc_s = tc_f.signal('jaMfa')")
        self.runCommand("tc_M = tc_s.value")
        self.runCommand("tc_M[0,3] = {}".format(self._translation[0]))
        self.runCommand("tc_M[1,3] = {}".format(self._translation[1]))
        self.runCommand("tc_M[2,3] = {}".format(self._translation[2]))
        self.runCommand("tc_s.value = tc_M")

    def parseSrdf(self):
        if not rospy.has_param("/demo"):
            rospy.logerr("No ROS parameter /demo")
            return
        srdf = {}
        # retrieve objects from ros param
        demoDict = rospy.get_param("/demo")
        robotDict = demoDict["robots"]
        if len(robotDict) != 1:
            raise RuntimeError("One and only one robot is supported for now.")
        objectDict = demoDict["objects"]
        objects = list(objectDict.keys())
        # parse robot and object srdf files
        srdfDict = dict()
        for r, data in robotDict.items():
            srdfDict[r] = parse_srdf(srdf = data["srdf"]["file"],
                                     packageName = data["srdf"]["package"],
                                     prefix=r)
        for o, data in objectDict.items():
            srdfDict[o] = parse_srdf(srdf = data["srdf"]["file"],
                                     packageName = data["srdf"]["package"],
                                     prefix=o)
        self.grippers = list(demoDict["grippers"])
        self.handles = list()
        for o in objects:
            self.handles.extend(sorted(list(srdfDict[o]["handles"].keys())))
