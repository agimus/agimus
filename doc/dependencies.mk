##### Agimus #######################################
agimus-demos.configure.dep:
agimus-vision.configure.dep:
agimus-hpp.configure.dep: hpp-corbaserver.install hpp-manipulation-corba.install \
	agimus_sot_msgs.install
agimus-sot.configure.dep: hpp-manipulation-corba.install agimus_sot_msgs.install
agimus.configure.dep: agimus-hpp.install agimus_sot_msgs.install
agimus_sot_msgs.configure.dep:
agimus-doc.configure.dep: agimus.install agimus-hpp.install agimus-sot.install

##### HPP Corba clients #######################################
hpp-manipulation-corba.configure.dep: hpp-corbaserver.install
hpp-corbaserver.configure.dep:

gepetto-viewer-corba.configure.dep:
hpp-gepetto-viewer.configure.dep: gepetto-viewer-corba.install \
	hpp-corbaserver.install

hpp-tools.configure.dep:

##### Robot models #######################################
robot_model_py.configure.dep:
robot_capsule_urdf.configure.dep: robot_model_py.install
hpp-hrp2.configure.dep: hrp2-14-description.install hpp-corbaserver.install
hrp2-14-description.configure.dep: robot_capsule_urdf.install \
	robot_model_py.install

iai_maps.configure.dep:

universal_robot.configure.dep:
hpp_universal_robot.configure.dep: universal_robot.install

hpp-environments.configure.dep:
baxter_common.configure.dep:
hpp-baxter.configure.dep: baxter_common.install
hpp_romeo.configure.dep: romeo.install
romeo.configure.dep:

gerard-bauzil.configure.dep:

hpp_tiago.configure.dep:

##### For simulation ######################################

rviz_camera_stream.configure.dep:
rviz_lighting.configure.dep:

##### For documentation ###################################

roslaunch_to_dot.configure.dep:
