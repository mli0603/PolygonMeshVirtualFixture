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

project (dvrk_mesh_vf)

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
     cisstMesh
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
find_package (OpenIGTLink 3
                HINTS igtl /usr/lib/igtl /usr/local/lib/igtl/cmake/igtl-3.1 /usr/local/lib/igtl /usr/local/)
if (cisst_FOUND)
  # load cisst configuration
  include (${CISST_USE_FILE})
  include (${OpenIGTLink_USE_FILE})

  # find include in catkin space
  include_directories (${catkin_INCLUDE_DIRS})
  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                CATKIN_DEPENDS cisst_ros_bridge geometry_msgs roscpp std_msgs)

  # catkin/ROS paths
  cisst_is_catkin_build (dvrk_mesh_vf_IS_CATKIN_BUILT)
  if (dvrk_mesh_vf_IS_CATKIN_BUILT)
    set (LIBRARY_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/lib")
  endif ()

  # create/configure file for find_package (dvrk_mesh_vf)
  set (dvrk_mesh_vf_INCLUDE_DIR
       "${dvrk_mesh_vf_SOURCE_DIR}/include"
       "${dvrk_mesh_vf_BINARY_DIR}/include")
  set (dvrk_mesh_vf_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (dvrk_mesh_vf_LIBRARIES dvrk_mesh_vf)

  # sawIntuitiveResearchKit has been compiled within cisst, we should
  # find it automatically
  find_package(sawIntuitiveResearchKit 1.7.1 REQUIRED)
  find_package(sawControllers 1.7.1 REQUIRED)
  find_package(sawConstraintController REQUIRED)
  find_package(sawOpenIGTLink REQUIRED)

  include_directories (
    ${CMAKE_BINARY_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    ${sawControllers_INCLUDE_DIR}
    ${sawConstraintController_INCLUDE_DIR}
    ${sawOpenIGTLink_INCLUDE_DIR})

  link_directories (
    ${sawIntuitiveResearchKit_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR}
    ${sawConstraintController_LIBRARY_DIR}
    ${sawOpenIGTLink_LIBRARY_DIR})

  set (dvrk_mesh_vf_INCLUDE_DIR
        "${dvrk_mesh_vf_SOURCE_DIR}/include")

  include_directories (${dvrk_mesh_vf_INCLUDE_DIR})

  add_library (dvrk_mesh_vf
              "include/mtsDerivedTeleOperationPSM.h"
              "code/mtsDerivedTeleOperationPSM.cpp"
              "include/mtsDerivedIntuitiveResearchKitPSM.h"
              "code/mtsDerivedIntuitiveResearchKitPSM.cpp"
              "include/robHelper.h"
              "code/mtsDerivedROSBridge.cpp"
              "include/mtsDerivedROSBridge.h"
              )

  add_executable(simpleTeleop
            simulation/main_teleop.cpp
            simulation/simple_teleop.cpp
            simulation/simple_teleop.h)

  cisst_target_link_libraries (dvrk_mesh_vf
                              ${REQUIRED_CISST_LIBRARIES})
  cisst_target_link_libraries (simpleTeleop
                              ${REQUIRED_CISST_LIBRARIES})

  # link against non cisst libraries and cisst components
  target_link_libraries (dvrk_mesh_vf
                        ${sawIntuitiveResearchKit_LIBRARIES}
                        ${sawControllers_LIBRARIES}
                        ${sawConstraintController_LIBRARIES}
                        ${sawOpenIGTLink_LIBRARIES})
  target_link_libraries (simpleTeleop
                        ${sawControllers_LIBRARIES}
                        ${sawConstraintController_LIBRARIES}
                        ${sawOpenIGTLink_LIBRARIES}
                        ${catkin_LIBRARIES})

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/dvrk_mesh_vfConfig.cmake.in"
                  "${dvrk_mesh_vf_CMAKE_CONFIG_FILE}"
                  @ONLY)
endif (cisst_FOUND)
