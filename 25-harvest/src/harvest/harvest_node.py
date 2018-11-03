#!/usr/bin/env python
# Description: node of arm manager
#    features:
#        provide higher level position/velocity/torque control
#    logic:
#        rosserial already have rx gcode, tx report.
#        
# Author: wuulong@gmail.com 
# Pubs: 
#    /arm_joint_states ( sensor_msgs/JointState ) # join_state
#    /arm_ctrl_state ( String? ) # status update system reply
#    /serial_tx ( std_msgs/String ) # string want to send to serial
# Subs:
#    /cmd_pos ( geometry_msgs/Point ) # position control
#    /cmd_vel ( geometry_msgs/Twist ) # velocity control
#    /serial_rx ( std_msgs/String ) # string need to receive from serial
# Actions:
#    FhbAct
# Reference: 
#    FarmBot G-code command list: https://github.com/FarmBot/farmbot-arduino-firmware#pin-numbering
# Test Hint:
#    rostopic pub -r 1 /cmd_pos geometry_msgs/Point '{ x: 0, y: 0, z: 0 }'
#    rostopic pub -r 1 /cmd_vel geometry_msgs/Twist '{ linear : {  x: 0, y: 0, z: 0 }, angular: {  x: 0, y: 0, z: 0 } }'
#    rostopic pub -r 1 /serial_rx std_msgs/String '{ data: R00 }'
#    rostopic echo /arm_ctrl_state


import rospy
import harvest
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

import actionlib
from actionlib_msgs.msg import GoalStatus
from farmharvestbot_msgs.msg import *
from fhb_utils import Consts,FhbActSrv


#FHB action server
class HarvestActSrv(FhbActSrv):
    # create messages that are used to publish feedback/result
    #_feedback = FhbActFeedback()
    #_result = FhbActResult()

    pass
# Rx /cmd_pos, Tx serial
# Rx serial, Tx /arm_ctrl_state reply to user
class HarvestNode():
    def __init__(self):
        self.fba = None # farmbot class
        self.actsrv = None 
        self.setup()
        
    def setup(self):
        self.ha = harvest.Harvest() 
        self.actsrv = HarvestActSrv("harvest_act")

    

# main process loop        
def main():
    rospy.init_node('harvest_node')
    rospy.loginfo("harvest_node started")

    hn = HarvestNode()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # get rx from rosserial, translate to ctrl_state
        rate.sleep() 
       

if __name__ == '__main__':
    main()