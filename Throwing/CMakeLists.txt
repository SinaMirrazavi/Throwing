cmake_minimum_required(VERSION 2.8.3)
project(fastthrowing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mathlib
  robotlib
  skinematics
  sgaussians
  vision_pkg
)




catkin_package(
  INCLUDE_DIRS FastThrowing/include
    INCLUDE_DIRS RandomThrowing/include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS mathlib robotlib
#  DEPENDS system_lib
)

include_directories(include)
 include_directories(FastThrowing/include)
  include_directories(RandomThrowing/include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_library(bimanual_catching
#   src/${PROJECT_NAME}/bimanual_catching.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(bimanual_catching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(Tester_Throwing_s src/FindOptimalJoints.cpp)
target_link_libraries(Tester_Throwing_s ${catkin_LIBRARIES})

add_executable(SkeletonStream src/SkeletonStreaming.cpp)
target_link_libraries(SkeletonStream ${catkin_LIBRARIES})

add_executable(SkeletonTest src/SkeletonTest.cpp)
target_link_libraries(SkeletonTest ${catkin_LIBRARIES})

add_executable(UpperIK src/UpperIK.cpp)
target_link_libraries(UpperIK ${catkin_LIBRARIES})

FILE(GLOB theSourceFiles_Fast FastThrowing/src/*.cpp)
add_library(FastThrowing ${theSourceFiles_Fast})
 target_link_libraries(FastThrowing
   ${catkin_LIBRARIES}
 )

 FILE(GLOB theSourceFiles_Random RandomThrowing/src/*.cpp)
add_library(RandomThrowing ${theSourceFiles_Random})
 target_link_libraries(RandomThrowing
   ${catkin_LIBRARIES}
 )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(bimanual_catching_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS bimanual_catching bimanual_catching_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bimanual_catching.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
###################################
## RobotToolKit specific stuff  ###
###################################
# to comply with the file structure of RobotToolKit, you need to create symlinks 
# to any compiled modules in the RobotToolKit/module folder. This is automated below.
# just add the modules of this package as set(rtk_modules myAwesomeModule1 myAwesomeModule2)
# modules can be world or robot modules. Note that you need to specify your targets using 
# standard cmake/catkin procedures above. This is step is only for linking the build modules.
set(rtk_modules FastThrowing RandomThrowing)
execute_process(COMMAND rospack find rtk_pkg_tools OUTPUT_VARIABLE rtk_pkg_tools_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
include(${rtk_pkg_tools_DIR}/rtk_create_symlinks.cmake)

# Uncomment the following line if you want catkin dependencies you specify in CMakeLists.txt to be automatically copied to the package.xml
execute_process(COMMAND ${rtk_pkg_tools_DIR}/scripts/CopyDepsFromCMakeListsToPackageXML.py ${PROJECT_NAME})










