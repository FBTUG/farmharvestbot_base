#!/usr/bin/env python
# Description: node of harvest manager
#    features:
#    logic:
#        rosserial already have rx gcode, tx report.
#        
# Author: wuulong@gmail.com 
# Pubs: 
# Subs:
# Actions:
#    FhbAct
# Statistic:
#    FhbStatistic
# Reference: 
# Test Hint:


import rospy
import harvest

import actionlib
from actionlib_msgs.msg import GoalStatus
from farmharvestbot_msgs.msg import *
from fhb_utils import Consts,FhbActSrv,FhbNode,FhbStatistic

import diagnostic_updater
import diagnostic_msgs

#harvest statistic
class HarvestStatistic(FhbStatistic):
        
    def local_init(self):
        self.fruit_cnt=1
        

            
    def stat_add(self,stat):
        stat.add("fruit_cnt", self.fruit_cnt)
        self.fruit_cnt= self.fruit_cnt+1
    def update(self):
        self.updater.update() #call when need to update statistic
        

#harvest action server
class HarvestActSrv(FhbActSrv):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FhbActAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

# Rx /cmd_pos, Tx serial
# Rx serial, Tx /arm_ctrl_state reply to user
class HarvestNode(FhbNode):
    def __init__(self):
        FhbNode.__init__(self,rospy.get_name())
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
    hs = HarvestStatistic("harvest")

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        # get rx from rosserial, translate to ctrl_state
        hs.update()
        rate.sleep() 
       

if __name__ == '__main__':
    main()