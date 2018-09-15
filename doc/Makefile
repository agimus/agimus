#
# Copyright (c) 2014 CNRS-LAAS
# Author: Florent Lamiraux
#

SRC_DIR=${DEVEL_HPP_DIR}/src
ifndef INSTALL_DIR
INSTALL_DIR=${DEVEL_HPP_DIR}/install
endif

# Whether to compute humanoid specific part
HUMANOID=TRUE

BUILD_TYPE=Release

INSTALL_DOCUMENTATION=ON

config.mk  = ${SRC_DIR}/agimus/doc/config.mk
targets.mk = ${SRC_DIR}/agimus/doc/targets.mk
ifeq ("$(wildcard ${config.mk})","")
	# File does not exist
  ifneq ($(MAKECMDGOALS),bootstrap)
	  $(error You must run target bootstrap first)
  endif
else
  include ${config.mk}
endif

all: agimus.install agimus-sot.install

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

hpp-util_branch=devel
hpp-util_repository=${HPP_REPO}

hpp-template-corba_branch=devel
hpp-template-corba_repository=${HPP_REPO}

hpp-corbaserver_branch=devel
hpp-corbaserver_repository=${HPP_REPO}
hpp-corbaserver_extra_flags=-DCLIENT_ONLY=TRUE

hpp-wholebody-step-corba_branch=devel
hpp-wholebody-step-corba_repository=${HPP_REPO}
hpp-wholebody-step-corba_extra_flags=-DCLIENT_ONLY=TRUE

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

romeo_branch=master
romeo_repository=${HPP_REPO}

ifeq (${HUMANOID}, TRUE)
  hpp-manipulation-corba_extra_flags+=-DHPP_MANIPULATION_HAS_WHOLEBODY_STEP=TRUE
else
  hpp-manipulation-corba_extra_flags+=-DHPP_MANIPULATION_HAS_WHOLEBODY_STEP=FALSE
endif

ifeq ("$(wildcard ${targets.mk})","")
	# File does not exist
  ifneq ($(MAKECMDGOALS),bootstrap)
	  $(error You must run target bootstrap first)
  endif
else
  include ${targets.mk}
endif
