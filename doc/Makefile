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
INSTALL_FROM_SOURCE     ?=FALSE
BUILD_TYPE              ?=Release
INSTALL_DOCUMENTATION   ?=ON

config.mk  = ${SRC_DIR}/agimus/doc/config.mk
targets.mk = ${SRC_DIR}/agimus/doc/targets.mk
dependencies.mk=${SRC_DIR}/agimus/doc/dependencies.mk
specific-rules.mk=${SRC_DIR}/agimus/doc/specific-rules.mk
sot_repos.mk = ${SRC_DIR}/agimus/doc/sot_from_src/repos.mk
sot_dependencies.mk = ${SRC_DIR}/agimus/doc/sot_from_src/dependencies.mk
sot_empty_dependencies.mk = ${SRC_DIR}/agimus/doc/sot_from_src/empty_dependencies.mk

ifeq ("$(wildcard ${config.mk})","")
	# File does not exist
  ifneq ($(MAKECMDGOALS),bootstrap)
	  $(error You must run target bootstrap first)
  endif
else
  include ${config.mk}
endif

ifeq (${INSTALL_FROM_SOURCE}, TRUE)
	include ${sot_repos.mk}
endif

agimus_branch=v1.0.0
agimus_repository=https://github.com/agimus

agimus-sot_branch=v1.0.0
agimus-sot_repository=${AGIMUS_REPO}

agimus_sot_msgs_branch=v1.0.0
agimus_sot_msgs_repository=${AGIMUS_REPO}

agimus-hpp_branch=v1.0.0
agimus-hpp_repository=${AGIMUS_REPO}
agimus-hpp_extra_flags=-DBUILD_ROS_INTERFACE=ON -DBUILD_HPP_PLUGIN=ON

agimus-vision_branch=v1.0.0
agimus-vision_repository=${AGIMUS_REPO}

agimus-demos_branch=v1.0.0
agimus-demos_repository=${AGIMUS_REPO}

agimus-doc_branch=v1.0.0
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

all: agimus.install agimus-sot.install rviz_camera_stream.install rviz_lighting.install

bootstrap:
	@if [ -d "agimus" ]; then \
		echo "Agimus already bootstrapped."; \
	else \
	  git clone --quiet --recursive -b ${agimus_branch} \
		${agimus_repository}/agimus.git; \
	fi

ifeq ("$(wildcard ${targets.mk})","")
       # File does not exist
  ifneq ($(MAKECMDGOALS),bootstrap)
         $(error You must run target bootstrap first)
  endif
else
  include ${dependencies.mk}
  ifeq (${INSTALL_FROM_SOURCE}, TRUE)
    include ${sot_dependencies.mk}
  else
    include ${sot_empty_dependencies.mk}
  endif
  include ${targets.mk}
  include ${specific-rules.mk}
endif
