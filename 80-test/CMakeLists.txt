cmake_minimum_required(VERSION 2.8.3)
project(farmharvestbot_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES farmharvestbot_arm 
  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)


#catkin_install_python(PROGRAMS src/arm.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


###########
## Build ##
###########




#############
## Install ##
#############



#############
## Testing ##
#############

