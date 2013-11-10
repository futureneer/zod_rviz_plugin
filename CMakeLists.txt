cmake_minimum_required(VERSION 2.8.3)
project(zod_rviz_plugin)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rviz
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
#######################################
## Declare ROS messages and services ##
#######################################

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

## QT
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## QT KEYWORDS
add_definitions(-DQT_NO_KEYWORDS)

###########
## Build ##
###########

## MOC HEADER FILES
qt4_wrap_cpp(MOC_FILES
  src/zod_display.h
  src/zod_visual.h
)

## SET SOURCE FILES
set(SOURCE_FILES
  src/zod_display.cpp 
  src/zod_visual.cpp 
  ${MOC_FILES}
)

## CREATE LIBRARIES
add_library(${PROJECT_NAME} ${SOURCE_FILES})

## LINK LIBRARIES
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

## INSTALL TARGETS
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## INSTALL PLUGIN
install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})