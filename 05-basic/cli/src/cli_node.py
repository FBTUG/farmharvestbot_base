#!/usr/bin/env python
# Description: CLI to service user.
#    features: 2 level commands
# Author: wuulong@gmail.com

import rospy
import rosnode
import rosbag
import rostest
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import String
#from farmharvestbot_msgs.msg import VersionInfo
import cmd
import time
import traceback
import numpy as np
import base_ut
from arm import Arm
from fhb_utils import Consts

VERSION = "0.0.2"
CMD_VERSION = "0.1"
PRJNAME="farmharvestbot_base"


# CLI root class, all CLI cmd based on this
# 2 level directory    
class RootCli(cmd.Cmd):
    def __init__(self):
        cmd.Cmd.__init__(self)
    # return [ True/False, [pars] ]
    #    True/False: input have errors or not
    #    pars: input parameters list
    def do_quit(self, line):
        """quit"""
        return True

    def _line_set(self,line, pars_default, str_not_match):
        pars = line.strip().split()
        ret = True
        if len(pars)< len(pars_default):
            rospy.loginfo(str_not_match)
            ret = False
            
        ret_par = pars_default
        for i in range(min(len(pars_default),len(pars))):
            ret_par[i] = pars[i]
        return [ret,ret_par]

#Main CLI class    
class BaseCli(RootCli):
    """Base CLI"""

    def __init__(self):
        cmd.Cmd.__init__(self)
        self.prompt = 'Base>'
        self.cmd_pub = None
        self.user_quit = False
        
        #sub level commands
        self.cli_sim = CliSim()
        self.cli_test = CliTest()
        self.cli_vision = CliVision()
        self.cli_arm = CliArm()
        self.cli_car = CliCar()

############ cli maintain ####################
    def do_quit(self, line):
        """quit"""
        self.user_quit = True
        return True
    #do_EOF = do_quit
    def do_version(self,line):
        """Report software version"""
        output = "V" + VERSION
        print(output)

############ sub level commands ####################            
    def do_vision(self,line):
        """simulation sub command directory"""
        self.cli_vision.prompt = self.prompt[:-1]+':vision>'
        self.cli_vision.cmdloop()

    def do_arm(self,line):
        """simulation sub command directory"""
        self.cli_arm.prompt = self.prompt[:-1]+':arm>'
        self.cli_arm.cmdloop()

    def do_car(self,line):
        """simulation sub command directory"""
        self.cli_car.prompt = self.prompt[:-1]+':car>'
        self.cli_car.cmdloop()

    def do_sim(self,line):
        """simulation sub command directory"""
        self.cli_sim.prompt = self.prompt[:-1]+':sim>'
        self.cli_sim.cmdloop()

    def do_test(self,line):
        """test sub command directory"""
        self.cli_test.prompt = self.prompt[:-1]+':test>'
        self.cli_test.cmdloop()

############ commands  ####################
#CLI-Vision level
class CliVision(RootCli):
    pass

#CLI-Arm level
class CliArm(RootCli):
    def __init__(self):
        cmd.Cmd.__init__(self)
        self.cmd_pos_pub=None
        self.arm_ctrl_state_sub=None
        
    def do_arm_setup(self,line):
        """ arm environment setup"""
        self.cmd_pos_pub = rospy.Publisher('/cmd_pos', Point, queue_size=1)
        self.arm_ctrl_state_sub = rospy.Subscriber('/arm_ctrl_state', String, self.cbArmCtrlState)
        rospy.loginfo("arm have been setup!" )

    def cbArmCtrlState(self,rx_str):
        rospy.loginfo("cli_node:receive from arm_ctrl_state: %s" %(rx_str))

                
    def do_cmd_pos(self,line):
        """ arm position control
cmd_pos [x] [y] [z]
ex: cmd_pos 10.0 20.0 5.0
    send /cmd_pos ( geometry_msgs/Point )
        """    
        [ret,pars] = self._line_set(line,["10.0","20.0","5.0"],"missing position information, use default: 10.0 20.0 5.0")
        if self.cmd_pos_pub:
            pnt = Point()
            pnt.x = float(pars[0])
            pnt.y = float(pars[1])
            pnt.z = float(pars[2])
            self.cmd_pos_pub.publish(pnt)

#CLI-Car level
class CliCar(RootCli):
    pass

#CLI-Test level
class CliTest(RootCli):
    def do_test_exception(self,line):
        """Test exception condition"""
        int("a")
    def do_test_tmp(self,line):
        """Current test"""
        arm = Arm()
        
        rospy.loginfo("ACTID_START_HARVET=%i" % Consts.ACTID_START_HARVEST)
        rospy.loginfo("arm.link_cnt=%i" % arm.link_cnt)

    def do_ut_example(self,line):
        """Unit test example"""

        rostest.rosrun('farmharvest_base', 'vision_tester_node', base_ut.VisionTesterNode)

#CLI-Sim level           
class CliSim(RootCli):

    def do_sim_setup(self,line):
        """Simulation environment setup"""
        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        rospy.loginfo("/turtle1/cmd_vel Publisher setup!" )
        

            
    def do_sim_turtlesim_key(self,line):
        """ demo turtlesim turtle_teleop_key
sim_turtlesim_key [keyname]
    keyname: U/D/L/R (UP/DOWN/LEFT/RIGHT)
ex: sim_turtlesim_key U
    send /turtle1/cmd_vel linear.x=1.0
        """    
        
        cmd = Twist()

        if not self.cmd_pub:   
            rospy.loginfo("sim_setup needed!" )
            return
        #rospy.loginfo("line=%s,len=%i" %(line,len(pars)))
        [ret,pars] = self._line_set(line,["U"],"missing keyname, use default: U")
        
        keyname = pars[0]
            
        if keyname == "D":
            cmd.linear.x=-1.0
        elif keyname == "R":
            cmd.angular.z=1.0
        elif keyname == "L":
            cmd.angular.z=-1.0
        else: #UP
            cmd.linear.x=1.0

        if self.cmd_pub:
            self.cmd_pub.publish(cmd)


def main():
    
    rospy.init_node('base_cli')
    rospy.loginfo("----- %s V%s" %(PRJNAME,str(VERSION)))
    rospy.loginfo("----- CLI need to run with ros master exist!" )
    bcli = BaseCli()
    while(1):
        try:
            
            bcli.cmdloop()
            if bcli.user_quit:
                break
            
            #rospy.spin()
        except (rospy.ROSInterruptException):
            rospy.loginfo("ROSInterruptException catch!")
            break
        #except ValueError:
        #    rospy.loginfo("ValueError catch!")
        except:
            traceback.print_exc()
            pass
       

if __name__ == '__main__':
    main()