cmake_minimum_required(VERSION 2.8.3)
project(farmharvestbot_harvest)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES farmharvestbot_harvest 
  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)



###########
## Build ##
###########




#############
## Install ##
#############



#############
## Testing ##
#############

