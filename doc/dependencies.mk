##### Agimus #######################################
agimus-demos.configure.dep: agimus-demos.checkout
agimus-vision.configure.dep: agimus-vision.checkout
agimus-hpp.configure.dep: hpp-corbaserver.install hpp-manipulation-corba.install \
	agimus_sot_msgs.install \
	agimus-hpp.checkout
agimus-sot.configure.dep: hpp-manipulation-corba.install agimus_sot_msgs.install \
	agimus-sot.checkout
agimus.configure.dep: agimus-hpp.install agimus_sot_msgs.install \
	agimus.checkout
agimus_sot_msgs.configure.dep: agimus_sot_msgs.checkout


##### HPP Corba clients #######################################
hpp-util.configure.dep: hpp-util.checkout
ifeq (${HUMANOID}, TRUE)
hpp-manipulation-corba.configure.dep: hpp-wholebody-step-corba.install \
	hpp-manipulation-corba.checkout
else
hpp-manipulation-corba.configure.dep: hpp-corbaserver.install \
	hpp-template-corba.install hpp-manipulation-corba.checkout
endif
hpp-corbaserver.configure.dep: hpp-corbaserver.checkout
hpp-wholebody-step-corba.configure.dep: hpp-corbaserver.install \
	hpp-template-corba.install \
	hpp-wholebody-step-corba.checkout
hpp-template-corba.configure.dep: hpp-util.install hpp-template-corba.checkout

gepetto-viewer-corba.configure.dep: gepetto-viewer-corba.checkout
hpp-gepetto-viewer.configure.dep: gepetto-viewer-corba.install \
	hpp-corbaserver.install \
	hpp-gepetto-viewer.checkout

hpp-tools.configure.dep: hpp-tools.checkout

##### Robot models #######################################
robot_model_py.configure.dep: robot_model_py.checkout
robot_capsule_urdf.configure.dep: robot_model_py.install \
	robot_capsule_urdf.checkout
hpp-hrp2.configure.dep: hrp2-14-description.install hpp-corbaserver.install \
	hpp-hrp2.checkout
hrp2-14-description.configure.dep: robot_capsule_urdf.install \
	robot_model_py.install hrp2-14-description.checkout

iai_maps.configure.dep: iai_maps.checkout

universal_robot.configure.dep: universal_robot.checkout
hpp_universal_robot.configure.dep: universal_robot.install \
	hpp_universal_robot.checkout

hpp-environments.configure.dep: hpp-environments.checkout
baxter_common.configure.dep: baxter_common.checkout
hpp-baxter.configure.dep: baxter_common.install hpp-baxter.checkout
hpp_romeo.configure.dep: romeo.install hpp_romeo.checkout
romeo.configure.dep: romeo.checkout
