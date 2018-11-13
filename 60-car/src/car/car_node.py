#!/usr/bin/env python
# Description: node of car manager
#    features:
#    logic:
#        
# Author: wuulong@gmail.com 
# Pubs: 
# Subs:
# Actions:
#    FhbAct
# Reference: 
# Test Hint:

import rospy
import vision

import actionlib
from actionlib_msgs.msg import GoalStatus
from farmharvestbot_msgs.msg import *
from fhb_utils import Consts,FhbActSrv,FhbNode,FhbStatistic

import diagnostic_updater
import diagnostic_msgs

#car statistic
class CarStatistic(FhbStatistic):
        
    def local_init(self):
        self.fruit_cnt=1
                 
    def stat_add(self,stat):
        stat.add("fruit_cnt", self.fruit_cnt)
        self.fruit_cnt= self.fruit_cnt+1
    def update(self):
        self.updater.update() #call when need to update statistic


#FHB action server
class CarActSrv(FhbActSrv):

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
        if goal.act_id>=Consts.ACTID_START_CAR:
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
        else:
            success = False
          
        if success:
            self.success_result("success")
        else:
            self.success_result("fail, not act_id not in valid range")
            
# car node
class CarNode(FhbNode):
    def __init__(self):
        FhbNode.__init__(self,rospy.get_name())
        self.vision = None # 
        
    def setup(self):
        self.vision = vision.Vision() 
    

# main process loop        
def main():
    rospy.init_node('car_node')
    rospy.loginfo("car_node started")

    cn = CarNode()
    cn.setup()
    server = CarActSrv("car_act")
    car_stat = CarStatistic("car")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        car_stat.update()
        rate.sleep() 
       

if __name__ == '__main__':
    main()