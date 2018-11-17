#!/usr/bin/env python
# Description: node of car manager
#    features:
#    logic:
#        
# Author: wuulong@gmail.com 
# Pubs: 
# Subs:
# Reference: 
# Test Hint:

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from farmharvestbot_msgs.msg import *
from fhb_utils import Consts,FhbActSrv,FhbNode

           
# car node
class WebTestNode(FhbNode):
    def __init__(self):
        pass
    def cbFeedback(self,feedback):
            rospy.loginfo("feedback: %s" %(feedback))

    def test(self):
        client = actionlib.SimpleActionClient('cli_act' , FhbActAction)
    
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
    
        # Creates a goal to send to the action server.
        goal = FhbActGoal()
        #a gold for debug purpose
        goal.act_id=201
        goal.act_subid=0 
        goal.i1=1
        goal.i2=2
        goal.f1=1.0
        goal.f2=2.0
        goal.line="!version"
         
    
        # Sends the goal to the action server. assign feedback callback
        # send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None)
        client.send_goal(goal,None,None,self.cbFeedback)
    
        # Waits for the server to finish performing the action.
        client.wait_for_result()
    
        # Prints out the result of executing the action
        rospy.loginfo( client.get_result())  # A FibonacciResult

    

# main process loop        
def main():
    rospy.init_node('web_node_test_node')
    rospy.loginfo("web_node_test started")

    wn = WebTestNode()
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        wn.test()
        rate.sleep() 
       

if __name__ == '__main__':
    main()