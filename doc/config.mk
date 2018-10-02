##### static variables ########
GEPGITLAB_REPO=https://gepgitlab.laas.fr/
GEPETTO_GEPGITLAB_REPO=${GEPGITLAB_REPO}/gepetto
PYRENE_GEPGITLAB_REPO=${GEPGITLAB_REPO}/pyrene-dev

AGIMUS_REPO=https://github.com/agimus
LAAS_REPO=https://github.com/laas
HPP_REPO=https://github.com/humanoid-path-planner
SOT_REPO=https://github.com/stack-of-tasks
RETHINK_ROBOTICS_REPO=https://github.com/RethinkRobotics
ROBOPTIM_REPO=https://github.com/roboptim
TRAC_REPO=ssh://trac.laas.fr/git/jrl/robots/ros-hrp2
PYRENE_DEV_REPO=git@gepgitlab.laas.fr:pyrene-dev

WGET=wget --quiet
UNZIP=unzip -qq
TAR=tar
GIT_QUIET=--quiet

define _status
  echo -n "\033[0;96m$1\033[0m"
endef
_msg_done=$(shell echo "\033[0;32mdone\033[0m")
_msg_warn=$(shell echo "\033[0;33mwarning\033[0m")

##### static variables ########

# Check that given variables are set and all have non-empty values,
# die with an error otherwise.
#
# Params:
#   1. Variable name(s) to test.
#   2. (optional) Error message to print.
check_defined = \
    $(strip $(foreach 1,$1, \
        $(call __check_defined,$1,$(strip $(value 2)))))
__check_defined = \
    $(if $(value $1),, \
      $(error Undefined $1$(if $2, ($2))))

$(call check_defined, SRC_DIR    , "source  directory")
$(call check_defined, INSTALL_DIR, "install directory")
$(call check_defined, BUILD_TYPE , "build type (Release or Debug)")
$(call check_defined, INSTALL_DOCUMENTATION, "ON or OFF")

ifeq (${BUILD_TYPE},Debug)
  BUILD_FOLDER=build
else
  BUILD_FOLDER=build-rel
endif
