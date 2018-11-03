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
# Reference: 
#    FarmBot G-code command list: https://github.com/FarmBot/farmbot-arduino-firmware#pin-numbering
# Test Hint:
#    rostopic pub -r 1 /cmd_pos geometry_msgs/Point '{ x: 0, y: 0, z: 0 }'
#    rostopic pub -r 1 /cmd_vel geometry_msgs/Twist '{ linear : {  x: 0, y: 0, z: 0 }, angular: {  x: 0, y: 0, z: 0 } }'
#    rostopic pub -r 1 /serial_rx std_msgs/String '{ data: R00 }'
#    rostopic echo /arm_ctrl_state


import rospy
import arm
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

import actionlib
from actionlib_msgs.msg import GoalStatus
from farmharvestbot_msgs.msg import *

STATUS_OK = "R00"
STATUS_CMDDONE="R02"

#FHB action server
class ArmActSrv(object):
    # create messages that are used to publish feedback/result
    _feedback = FhbActFeedback()
    _result = FhbActResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FhbActAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # setup feedback msg
        self._feedback.notice_id = 1
        self._feedback.progrsss=10
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing...' % (self._action_name))
        
        # start executing the action
        if goal.act_id>=200:
            for i in range(5):
                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._feedback.progrsss = i*20
                # publish the feedback
                self._as.publish_feedback(self._feedback)
                # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
                r.sleep()
          
        if success:
            # reply a result for debug purpose
            self._result.status = GoalStatus.SUCCEEDED #1 # 0-ok, >1 error code
            self._result.error_level =2 # 0-warning, 1-error, 2-fatal error 
            self._result.error_msg ="success"
            
            rospy.loginfo('%s: Succeeded' % self._action_name)
            
            self._as.set_succeeded(self._result)
            
# Rx /cmd_pos, Tx serial
# Rx serial, Tx /arm_ctrl_state reply to user
class ArmNode():
    def __init__(self):
        self.fba = None # farmbot class
        self.arm_ctrl_state_pub = None # arm_statusupdate publisher
        self.serial_rx_pub = None # serial rx publisher
        self.cmd_pos_cnt=0
        
    def setup(self):
        self.fba = arm.FarmBotArm() 
        self.arm_ctrl_state_pub = rospy.Publisher('arm_ctrl_state', String, queue_size=1)
        self.serial_rx_pub = rospy.Publisher('serial_rx', String, queue_size=1)
        rospy.Subscriber('cmd_pos', Point, self.cbCmdPos)
        rospy.Subscriber('cmd_vel', Twist, self.cbCmdVel)
        rospy.Subscriber('serial_tx', String, self.cbSerialRx)
    
    def cbCmdPos(self,pos_cmd):
        #str_cmd = "\t".join(['%s:: %s' % (key, value) for (key, value) in pos_cmd.items()])
        rospy.loginfo("Be request to run position command: %s" %(pos_cmd))
        self.cmd_pos_cnt = self.cmd_pos_cnt + 1
        
        x = 0
        y = 0
        z = 0
        gcode_str = "G01 %s,%s,%s" %(x,y,z)
        self.serial_rx_pub.publish(gcode_str)
    def cbCmdVel(self,vel_cmd):
        rospy.loginfo("Be request to run velocity command: %s" %(vel_cmd))
        gcode_str = "G01 %s,%s,%s"
        self.serial_rx_pub(gcode_str)
    def cbSerialRx(self,rx_str):
        rospy.loginfo("arm: receive from serial: %s" %(rx_str.data))
        # handle rx to see if need to do something
        if rx_str.data == STATUS_CMDDONE:
            #rospy.loginfo("arm: detect STATUS_CMDDONE")
            self.pub_arm_ctrl_state("DONE")
    def pub_arm_ctrl_state(self,line):
        self.arm_ctrl_state_pub.publish(line)

# main process loop        
def main():
    rospy.init_node('arm_node')
    rospy.loginfo("arm_node started")

    an = ArmNode()
    an.setup()
    server = ArmActSrv("arm_act")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # get rx from rosserial, translate to ctrl_state
        rate.sleep() 
       

if __name__ == '__main__':
    main()