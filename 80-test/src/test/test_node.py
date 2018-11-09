#!/usr/bin/env python
# Description: node of test manager
#    test cases:
#        test_act_arch
#        
# Author: wuulong@gmail.com 
# Test Hint:


import rospy
import rosbag
import roslib 
import unittest,rostest
import sys
from fhb_utils import FhbMisc

## base test node
class TesterNode(unittest.TestCase):

    def __init__(self, *args):
        super(TesterNode, self).__init__(*args)
        self.setup()

    def setup(self):
        # Setup the node
        rospy.init_node('tester_node', anonymous=False)
        self.fm = FhbMisc()
        self.pkg_dir = "%s" %(self.fm.get_package_dir('farmharvestbot_test')) 
        self.results_dir = rospy.get_param("/fhb_test/results_dir",self.pkg_dir)
        rospy.loginfo("get_param results_dir=%s" %(self.results_dir))
        if self.results_dir == "": #default
            self.results_dir = "%s/results" %(self.pkg_dir)
        #self.results_dir = "%s/results" %(self.pkg_dir)

    def test_act_arch(self):    
        """
        test case: action framework architecuture test, check if action framework basic normal
        input: bag/act_arch.bag
        output: results/r_act_arch.bag
        check: if result bag have received /harvest_act/result msg
        """    
        r_bagname = "r_act_arch.bag"
        r_bagpath = "%s/%s" %(self.results_dir,r_bagname)
        
        cmd_record="rosbag record --duration=17 -O %s -e '/harvest_act/result'" %(r_bagpath)
        cmd_play="rosbag play %s/bag/act_arch.bag --topics /harvest_act/goal" %(self.pkg_dir)

        #this test need to manual setup record duration, the duration need > bag play time + response time of expacted behavior
        #this example need ./big.bag 

        self.fm.bag_play_record(cmd_record,cmd_play,True)
        
        
        #check there are records in recorded file
        msg_exist = self.fm.bag_have_msg_from_topic(r_bagpath,"/harvest_act/result")
        
        self.assertEquals(msg_exist, True, "action result not received")


    
# main process loop        
def main():
    rostest.rosrun('farmharvestbot_test', 'tester_node', TesterNode)


if __name__ == '__main__':
    main()