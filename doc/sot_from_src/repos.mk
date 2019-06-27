eigenpy_branch=master
eigenpy_repository=${SOT_REPO}

pinocchio_branch=v2.1.3
pinocchio_repository=${SOT_REPO}
pinocchio_extra_flags=-DBUILD_UNIT_TESTS=OFF

dynamic-graph_branch=master
dynamic-graph_repository=${SOT_REPO}

dynamic-graph-python_branch=master
dynamic-graph-python_repository=${SOT_REPO}

dynamic_graph_bridge_branch=master
dynamic_graph_bridge_repository=${SOT_REPO}

dynamic_graph_bridge_msgs_branch=master
dynamic_graph_bridge_msgs_repository=${SOT_REPO}

sot-core_branch=master
sot-core_repository=${SOT_REPO}

sot-tools_branch=master
sot-tools_repository=${SOT_REPO}

sot-dynamic-pinocchio_branch=master
sot-dynamic-pinocchio_repository=${SOT_REPO}

roscontrol_sot_branch=master
roscontrol_sot_repository=${SOT_REPO}

sot-talos_branch=master
sot-talos_repository=${SOT_REPO}

talos_data_branch=master
talos_data_repository=${PYRENE_DEV_REPO}

talos_metapkg_ros_control_sot_branch=master
talos_metapkg_ros_control_sot_repository=${SOT_REPO}
talos_metapkg_ros_control_sot_subdirs=sot_pyrene_bringup
talos_metapkg_ros_control_sot_subdirs+=roscontrol_sot_talos
talos_metapkg_ros_control_sot_subdirs+=talos_metapkg_ros_control_sot

sot-tiago_branch=master
sot-tiago_repository=${SOT_REPO}

tiago_data_branch=master
tiago_data_repository=${SOT_REPO}

tiago_metapkg_ros_control_sot_branch=master
tiago_metapkg_ros_control_sot_repository=${SOT_REPO}
tiago_metapkg_ros_control_sot_subdirs=sot_tiago_bringup
tiago_metapkg_ros_control_sot_subdirs+=roscontrol_sot_tiago
tiago_metapkg_ros_control_sot_subdirs+=tiago_metapkg_ros_control_sot

include agimus/doc/sot_from_src/dependencies.mk
include agimus/doc/sot_from_src/robotpkg.mk
