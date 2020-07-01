##### Agimus #######################################
agimus-demos.configure.dep:
agimus-vision.configure.dep:
agimus-hpp.configure.dep: dynamic_graph_bridge_msgs.install \
	agimus_sot_msgs.install
agimus-sot.configure.dep: dynamic_graph_bridge.install agimus_sot_msgs.install
agimus.configure.dep: agimus-hpp.install agimus_sot_msgs.install
agimus_sot_msgs.configure.dep:
agimus-doc.configure.dep: agimus.install agimus-hpp.install agimus-sot.install

gerard-bauzil.configure.dep:

hpp_tiago.configure.dep:

##### For simulation ######################################

rviz_camera_stream.configure.dep:
rviz_lighting.configure.dep:

##### For documentation ###################################

roslaunch_to_dot.configure.dep:
