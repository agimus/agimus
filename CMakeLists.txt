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

set(PROJECT_NAME agimus)
set(PROJECT_DESCRIPTION "Software framework for manipulation planning and execution.")
set(PROJECT_URL "")
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_EXPORT_NO_TARGET TRUE)

include(cmake/base.cmake)
include(cmake/python.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

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
add_project_dependency(agimus-hpp)
add_project_dependency(agimus_sot_msgs)

ADD_SUBDIRECTORY(src)

INSTALL(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

INSTALL(PROGRAMS
  scripts/path_execution_fsm.py
  scripts/rqt_path_execution
  scripts/rqt_tooltip_calibration
  scripts/ros_synchro
  scripts/simulate_time
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
INSTALL(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
