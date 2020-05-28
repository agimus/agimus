#
# Copyright (c) 2018 CNRS-LAAS
# Author: Joseph Mirabel
#


ifndef DEVEL_HPP_DIR
$(error Environment variable DEVEL_HPP_DIR not defined. Did you source config.sh ?)
endif

SRC_DIR=${DEVEL_HPP_DIR}/src
ifndef INSTALL_DIR
INSTALL_DIR=${DEVEL_HPP_DIR}/install
endif

# If you have a SSH key:
# GEPGITLAB_WEBSITE=git@gepgitlab.laas.fr:
# GITHUB_WEBSITE=git@github.com:

# Install hpp and sot from source
# INSTALL_FROM_SOURCE=TRUE

BUILD_TYPE=Release

INSTALL_DOCUMENTATION=ON

config.mk  = ${SRC_DIR}/agimus/doc/config.mk
targets.mk = ${SRC_DIR}/agimus/doc/targets.mk
sot_repos.mk = ${SRC_DIR}/agimus/doc/sot_from_src/repos.mk
ifeq ("$(wildcard ${config.mk})","")
	# File does not exist
  ifneq ($(MAKECMDGOALS),bootstrap)
	  $(error You must run target bootstrap first)
  endif
else
  include ${config.mk}
endif

all: agimus.install agimus-sot.install rviz_camera_stream.install rviz_lighting.install

do-nothing:
ifneq (${INSTALL_FROM_SOURCE}, "TRUE")
dynamic-graph.install: do-nothing
dynamic-graph-python.install: do-nothing
dynamic_graph_bridge.install: do-nothing
dynamic_graph_bridge_msgs.install: do-nothing
sot-core.install: do-nothing
sot-tools.install: do-nothing
sot-dynamic-pinocchio.install: do-nothing
roscontrol_sot.install: do-nothing
sot-talos.install: do-nothing
talos_data.install: do-nothing
talos_metapkg_ros_control_sot.install: do-nothing
sot-tiago.install: do-nothing
tiago_data.install: do-nothing
tiago_metapkg_ros_control_sot.install: do-nothing
endif

bootstrap:
	@if [ -d "agimus" ]; then \
		echo "Agimus already bootstrapped."; \
	else \
	  git clone --quiet --recursive https://github.com/agimus/agimus.git; \
	fi

agimus_branch=master
agimus_repository=${AGIMUS_REPO}

agimus-sot_branch=master
agimus-sot_repository=${AGIMUS_REPO}

agimus_sot_msgs_branch=master
agimus_sot_msgs_repository=${AGIMUS_REPO}

agimus-hpp_branch=master
agimus-hpp_repository=${AGIMUS_REPO}
agimus-hpp_extra_flags=-DBUILD_ROS_INTERFACE=ON

agimus-vision_branch=master
agimus-vision_repository=${AGIMUS_REPO}

agimus-demos_branch=master
agimus-demos_repository=${AGIMUS_REPO}

agimus-doc_branch=master
agimus-doc_repository=${AGIMUS_REPO}

gerard-bauzil_branch=master
gerard-bauzil_repository=${GEPETTO_GEPGITLAB_REPO}

hpp_tiago_branch=master
hpp_tiago_repository=${HPP_REPO}

rviz_camera_stream_branch=master
rviz_camera_stream_repository=${AGIMUS_REPO}

rviz_lighting_branch=master
rviz_lighting_repository=${AGIMUS_REPO}

roslaunch_to_dot_branch=master
roslaunch_to_dot_repository=https://github.com/bponsler

ifeq (${INSTALL_FROM_SOURCE}, TRUE)
	include ${sot_repos.mk}
endif

ifeq ("$(wildcard ${targets.mk})","")
	# File does not exist
  ifneq ($(MAKECMDGOALS),bootstrap)
	  $(error You must run target bootstrap first)
  endif
else
  include ${targets.mk}
endif
