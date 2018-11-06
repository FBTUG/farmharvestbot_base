#!/usr/bin/env python

import rospy
import roslib 
import unittest,rostest
import sys



## A sample python unit test
class VisionTesterNode(unittest.TestCase):

    def __init__(self, *args):
        super(VisionTesterNode, self).__init__(*args)
        self.msg_received = False


    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        self.assertEquals(1, 1, "1!=1")

    
if __name__ == '__main__':
    #rospy.init_node('vision_tester_node', anonymous=False)
    PKG = 'farmharvest_base'
    rostest.rosrun(PKG, 'vision_tester_node', VisionTesterNode)