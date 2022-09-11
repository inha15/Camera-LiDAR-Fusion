cmake_minimum_required(VERSION 3.0.2)
project(Sensor_fusion) 

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
   rospy
   cv_bridge
   image_transport
)

add_message_files(
  FILES
  obj_msg.msg
  LiDAR_BB.msg
  LiDAR_BB_arr.msg
  Camera_BB.msg
  Camera_BB_arr.msg
  object_msg.msg
  object_msg_arr.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp std_msgs rospy cv_bridge image_transport message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(Camera_proc src/fusion_func/Camera_processing.cpp)
target_link_libraries(Camera_proc ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(LiDAR_proc src/fusion_func/LiDAR_processing.cpp)
target_link_libraries(LiDAR_proc ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(fusion_proc src/fusion_func/fusion_processing.cpp)
target_link_libraries(fusion_proc ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(msg_generator src/fusion_func/fusion_msg.cpp)
target_link_libraries(msg_generator ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(set_calibmat src/fusion_func/CalibMat_setting.cpp)
target_link_libraries(set_calibmat ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(only_deliv src/fusion_func/delivery_mission.cpp)
target_link_libraries(only_deliv ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} Camera_proc LiDAR_proc fusion_proc msg_generator set_calibmat)

add_library(sync_proc src/fusion_func/sync_processing.cpp)
target_link_libraries(sync_proc ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} LiDAR_proc Camera_proc fusion_proc msg_generator set_calibmat only_deliv)


add_executable(Data_RCVD src/fusion_node/Data_Receive.cpp)
target_link_libraries(Data_RCVD sync_proc)

#add_executable(Final_fusion src/fusion_node/final_fusion.cpp)
#target_link_libraries(Final_fusion Camera_proc LiDAR_proc fusion_proc)

