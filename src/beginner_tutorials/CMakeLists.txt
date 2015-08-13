cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  sensor_msgs
  message_filters
  image_transport
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Num.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories( ${catkin_INCLUDE_DIRS} )

# --------------------------------------------------
# Adding a Talker
# --------------------------------------------------
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker ${catkin_EXPORTED_TARGETS})

# --------------------------------------------------
# Adding a listener
# --------------------------------------------------
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${catkin_EXPORTED_TARGETS}) 

# --------------------------------------------------
# Adding a writedepthimage listener
# --------------------------------------------------
add_executable(writedepthimage src/writedepthimage.cpp)
target_link_libraries(writedepthimage ${catkin_LIBRARIES})
add_dependencies(writedepthimage ${catkin_EXPORTED_TARGETS}) 

# --------------------------------------------------
# Adding a writergbdimages listener
# --------------------------------------------------
add_executable(writergbdimages src/writergbdimages.cpp)
target_link_libraries(writergbdimages ${catkin_LIBRARIES})
add_dependencies(writergbdimages ${catkin_EXPORTED_TARGETS}) 
