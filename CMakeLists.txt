# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-corbaserver.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-corbaserver is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME agimus)
SET(PROJECT_DESCRIPTION "Software framework for manipulation planning and execution.")
SET(PROJECT_URL "")

SETUP_PROJECT()
FINDPYTHON(2.7 REQUIRED EXACT)

find_package(catkin REQUIRED COMPONENTS
  std_srvs std_msgs
  agimus_sot_msgs
  smach_ros rospy
  qt_gui qt_gui_py_common
  rqt_gui rqt_gui_py)

catkin_package()

# Required for the ros_tools Python module and the Python class HppClient
# Should it only be agimus_hpp_msgs ?
ADD_REQUIRED_DEPENDENCY("agimus_hpp")
ADD_REQUIRED_DEPENDENCY("agimus_sot_msgs")

ADD_SUBDIRECTORY(src)

INSTALL(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

INSTALL(PROGRAMS
  scripts/path_execution_fsm.py
  scripts/rqt_path_execution
  scripts/ros_synchro
  scripts/simulate_time
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
INSTALL(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

SETUP_PROJECT_FINALIZE()
