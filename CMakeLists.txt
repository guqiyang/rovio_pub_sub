cmake_minimum_required(VERSION 2.8.3)
project(rovio_pub)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
 add_message_files(
 FILES 
  manDrv.msg
  report.msg
  Image.msg
   )
  generate_messages(
   DEPENDENCIES
  std_msgs
  sensor_msgs
 )
 
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rovio_pub
 CATKIN_DEPENDS  message_runtime
#  DEPENDS system_lib

)
## Generate added messages and services with any dependencies listed here
include_directories(include
  ${catkin_INCLUDE_DIRS}
)
add_library(Parser src/Parser.cpp)

##declare the cpp file
#add_executable(Parser src/Parser.cpp)

add_executable(rovioBase_pub src/rovioBase_pub.cpp) 
add_executable(roviotest src/roviotest.cpp) 

add_dependencies(rovioBase_pub rovio_pub_gencpp)
add_dependencies(roviotest rovio_pub_gencpp)

target_link_libraries(Parser ${catkin_LIBRARIES} ${CURL_LIBRARIES} ${OpenCV_LIBRARIES})

target_link_libraries(rovioBase_pub Parser) 
target_link_libraries(roviotest Parser)  
