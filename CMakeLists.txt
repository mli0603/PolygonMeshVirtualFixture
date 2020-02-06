#
# (C) Copyright 2011-2017 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (USAblation)

cmake_minimum_required(VERSION 2.8)

find_package(catkin REQUIRED COMPONENTS
        cisst_ros_bridge
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)
  # load cisst configuration
  include (${CISST_USE_FILE})

  # find include in catkin space
  include_directories (${catkin_INCLUDE_DIRS})
  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                CATKIN_DEPENDS cisst_ros_bridge geometry_msgs roscpp std_msgs)

  # catkin/ROS paths
  cisst_is_catkin_build (USAblation_IS_CATKIN_BUILT)
  if (USAblation_IS_CATKIN_BUILT)
    set (LIBRARY_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/lib")
  endif ()

  # create/configure file for find_package (USAblation)
  set (USAblation_INCLUDE_DIR
       "${USAblation_SOURCE_DIR}/include"
       "${USAblation_BINARY_DIR}/include")
  set (USAblation_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (USAblation_LIBRARIES USAblation)

  # sawIntuitiveResearchKit has been compiled within cisst, we should
  # find it automatically
  find_package(sawIntuitiveResearchKit 1.7.1 REQUIRED)
  find_package(sawControllers 1.7.1 REQUIRED)
  find_package(sawConstraintController REQUIRED)
  find_package(cisstICP REQUIRED)

  include_directories (
    ${CMAKE_BINARY_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    ${sawControllers_INCLUDE_DIR}
    ${sawConstraintController_INCLUDE_DIR}
    ${cisstICP_INCLUDE_DIR})

  link_directories (
    ${sawIntuitiveResearchKit_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR}
    ${sawConstraintController_LIBRARY_DIR}
    ${cisstICP_LIBRARY_DIR})

  set (USAblation_INCLUDE_DIR
        "${USAblation_SOURCE_DIR}/include")

  include_directories (${USAblation_INCLUDE_DIR})

  add_library (USAblation
              "include/mtsDerivedTeleOperationPSM.h"
              "code/mtsDerivedTeleOperationPSM.cpp"
              "include/mtsDerivedIntuitiveResearchKitPSM.h"
              "code/mtsDerivedIntuitiveResearchKitPSM.cpp"
              "include/robHelper.h"
              "code/mtsDerivedROSBridge.cpp"
              "include/mtsDerivedROSBridge.h"
              )

  cisst_target_link_libraries (USAblation
                              ${REQUIRED_CISST_LIBRARIES})

  # link against non cisst libraries and cisst components
  target_link_libraries (USAblation
                        ${sawIntuitiveResearchKit_LIBRARIES}
                        ${sawControllers_LIBRARIES}
                        ${sawConstraintController_LIBRARIES}
                        ${cisstICP_LIBRARIES})

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/USAblationConfig.cmake.in"
                    "${USAblation_CMAKE_CONFIG_FILE}"
                    @ONLY)
endif (cisst_FOUND)
