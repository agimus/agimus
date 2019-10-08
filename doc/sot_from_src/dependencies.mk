##### Pinocchio #######################################
eigenpy.configure.dep:
pinocchio.configure.dep: eigenpy.install

##### Dynamic graph #######################################
dynamic-graph.configure.dep:
dynamic-graph-python.configure.dep: dynamic-graph.install

dynamic_graph_bridge_msgs.configure.dep:
dynamic_graph_bridge.configure.dep: dynamic-graph.install \
	dynamic_graph_bridge_msgs.install \
	dynamic-graph-python.install \
	sot-dynamic-pinocchio.install \
	sot-core.install

##### SoT #######################################
sot-core.configure.dep: pinocchio.install dynamic-graph-python.install
sot-tools.configure.dep: sot-core.install
sot-dynamic-pinocchio.configure.dep: pinocchio.install \
	sot-core.install sot-tools.install
roscontrol_sot.configure.dep: dynamic_graph_bridge.install \
	sot-core.install sot-dynamic-pinocchio.install

##### Pyrene #######################################
talos_data.configure.dep:
sot-talos.configure.dep: dynamic_graph_bridge.install \
	sot-dynamic-pinocchio.install \
	talos_data.install
talos_metapkg_ros_control_sot.configure.dep: roscontrol_sot.install \
	sot-talos.install

##### Tiago #######################################
tiago_data.configure.dep:
sot-tiago.configure.dep: dynamic_graph_bridge.install \
	sot-dynamic-pinocchio.install \
	tiago_data.install
tiago_metapkg_ros_control_sot.configure.dep: roscontrol_sot.install \
	sot-tiago.install
