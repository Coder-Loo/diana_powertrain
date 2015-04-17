cmake_minimum_required(VERSION 2.8.3)
project(diana_powertrain)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX1Y)
if(COMPILER_SUPPORTS_CXX1Y)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++1y (C++14) support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  team_diana_lib
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system coroutine program_options)
find_package(hlcanopen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES diana_powertrain
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${hlcanopen_INCLUDE_DIRS}
  include/
)

## Declare a cpp library
# add_library(diana_powertrain
#   src/${PROJECT_NAME}/diana_powertrain.cpp
# )

## Declare a cpp executable

add_library(commons
  src/utils.cpp
)

add_library(pci7841_card
  src/pci7841_card.cpp
)

target_link_libraries(pci7841_card
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${hlcanopen_LIBRARIES}
  pci_7841
  commons
)

add_executable(diana_powertrain_node
  src/diana_powertrain_node.cpp
)

target_link_libraries(diana_powertrain_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${hlcanopen_LIBRARIES}
   pci7841_card
   commons
 )

add_executable(test_motor
  src/test_motor.cpp
)

target_link_libraries(test_motor
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${hlcanopen_LIBRARIES}
   pci7841_card
   commons
 )

add_library(command_line
  src/command_line.cpp
)

target_link_libraries(command_line
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${hlcanopen_LIBRARIES}
   pci7841_card
   commons
 )

add_executable(shell
  src/shell.cpp
)

target_link_libraries(shell
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${hlcanopen_LIBRARIES}
   pci7841_card
   command_line
   commons
 )

install(TARGETS pci7841_card
 DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS diana_powertrain_node
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS test_motor
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS command_line
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS shell
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_diana_powertrain.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
