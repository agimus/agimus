##### Pinocchio #######################################
eigenpy.configure.dep:
pinocchio.configure.dep: eigenpy.install

##### Dynamic graph #######################################
dynamic-graph.configure.dep:
dynamic-graph-python.configure.dep: dynamic-graph.install

dynamic_graph_bridge_msgs.configure.dep:
dynamic_graph_bridge.configure.dep: dynamic-graph.install \
	dynamic_graph_bridge_msgs.install \
	dynamic-graph-python.install \
	sot-core.install

##### SoT #######################################
sot-core.configure.dep: dynamic-graph-python.install
sot-tools.configure.dep: sot-core.install
sot-dynamic-pinocchio.configure.dep: pinocchio.install \
	sot-core.install sot-tools.install

##### Pyrene #######################################
talos_data.configure.dep:
sot-talos.configure.dep: dynamic_graph_bridge.install \
	sot-dynamic-pinocchio.install \
	talos_data.install
