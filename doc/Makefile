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

# For SoT developpers only
# INSTALL_SOT_FROM_SOURCE=TRUE

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

all: agimus.install agimus-sot.install hpp-gepetto-viewer.install rviz_camera_stream.install rviz_lighting.install

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

agimus-vision_branch=master
agimus-vision_repository=${AGIMUS_REPO}

agimus-demos_branch=master
agimus-demos_repository=${AGIMUS_REPO}

agimus-doc_branch=master
agimus-doc_repository=${AGIMUS_REPO}

hpp-corbaserver_branch=devel
hpp-corbaserver_repository=${HPP_REPO}
hpp-corbaserver_extra_flags=-DCLIENT_ONLY=TRUE

hrp2-14-description_branch=master
hrp2-14-description_repository=${TRAC_REPO}

hpp-hrp2_branch=devel
hpp-hrp2_repository=${HPP_REPO}

robot_capsule_urdf_branch=groovy
robot_capsule_urdf_repository=${LAAS_REPO}

robot_model_py_branch=groovy
robot_model_py_repository=${LAAS_REPO}

hpp-manipulation-corba_branch=devel
hpp-manipulation-corba_repository=${HPP_REPO}
hpp-manipulation-corba_extra_flags=-DCLIENT_ONLY=TRUE

hpp-tools_branch=master
hpp-tools_repository=${HPP_REPO}
hpp-tools_extra_flags=

iai_maps_branch=devel
iai_maps_repository=${HPP_REPO}

gepetto-viewer-corba_branch=devel
gepetto-viewer-corba_repository=${HPP_REPO}
gepetto-viewer-corba_extra_flags=-DCLIENT_ONLY=TRUE
ifeq (${QT_VERSION}, 5)
  gepetto-viewer-corba_extra_flags+= -DUSE_QT4=OFF
endif

hpp-gepetto-viewer_branch=devel
hpp-gepetto-viewer_repository=${HPP_REPO}

hpp-environments_branch=devel
hpp-environments_repository=${HPP_REPO}

universal_robot_branch=kinetic
universal_robot_repository=${HPP_REPO}

hpp_universal_robot_branch=devel
hpp_universal_robot_repository=${HPP_REPO}

baxter_common_branch=master
baxter_common_repository=${RETHINK_ROBOTICS_REPO}

hpp-baxter_branch=master
hpp-baxter_repository=${HPP_REPO}

hpp_romeo_branch=master
hpp_romeo_repository=${HPP_REPO}

gerard-bauzil_branch=master
gerard-bauzil_repository=${GEPETTO_GEPGITLAB_REPO}

romeo_branch=master
romeo_repository=${HPP_REPO}

hpp_tiago_branch=master
hpp_tiago_repository=${HPP_REPO}

rviz_camera_stream_branch=master
rviz_camera_stream_repository=${AGIMUS_REPO}

rviz_lighting_branch=master
rviz_lighting_repository=${AGIMUS_REPO}

roslaunch_to_dot_branch=master
roslaunch_to_dot_repository=https://github.com/bponsler

ifeq (${INSTALL_SOT_FROM_SOURCE}, TRUE)
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
