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
from diagnostic_msgs.msg import DiagnosticArray
#from farmharvestbot_msgs.msg import VersionInfo
import cmd
import time
import traceback
import numpy as np
import base_ut
from arm import Arm


import actionlib
from actionlib_msgs.msg import GoalStatus
from farmharvestbot_msgs.msg import *
from fhb_utils import Consts,FhbActSrv,FhbNode

import subprocess, os, signal

VERSION = "0.1.2"
CMD_VERSION = "0.1"
PRJNAME="farmharvestbot_base"

ACTID_CLI_CMD=1


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
        self.cli_sys = CliSys()
        self.cli_harvest = CliHarvest()
        self.cli_vision = CliVision()
        self.cli_arm = CliArm()
        self.cli_car = CliCar()
        
        self.cas_mode=0 #0-normal mode, 1: json mode
        self.cas= CliActSrv("cli_act")
        self.cas.assign_cmdfun(self.actioncmd)
        self.cas_ret = []

############ cli maintain ####################

    def do_quit(self, line):
        """quit"""      
        self.user_quit = True
        return True
    #do_EOF = do_quit
    def do_version(self,line):
        """Report software version"""
        out = "V%s" %(VERSION)
        rospy.loginfo(out)
        if self.cas_mode == 1:
            #json_str = "{\n \"%s\" : \"%s\"\n}" %("response",out)            
            json_str = '{ "%s" : "%s"}' %("response",out)            
            return json_str
        else:
            return out
    # process cmd from actionserver
    # line start by ! will return json format
    def actioncmd(self,line):
        rospy.loginfo("actioncmd:%s" %(line))
        self.cas_mode=0
        if len(line)>1 and line[0]=="!":
            self.cas_mode=1 #json
            line = line[1:]
        #self.cas_ret = []
        str_ret = self.onecmd(line)
        if str_ret:
            return str_ret
        else:
            return '!KO'
        #return "\n".join(self.cas_ret)
        

############ sub level commands ####################            
    def do_sys(self,line):
        """sys sub command directory"""
        self.cli_sys.prompt = self.prompt[:-1]+':sys>'
        self.cli_sys.cmdloop()

    def do_vision(self,line):
        """simulation sub command directory"""
        self.cli_vision.prompt = self.prompt[:-1]+':vision>'
        self.cli_vision.cmdloop()

    def do_harvest(self,line):
        """simulation sub command directory"""
        self.cli_harvest.prompt = self.prompt[:-1]+':harvest>'
        self.cli_harvest.cmdloop()

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
#CLI-Sysstem level
class CliSys(RootCli):
    def __init__(self):
        cmd.Cmd.__init__(self)
        self.stat_sub = None
    def do_monitor_statistic(self,line):
        """ sys monitor statistic example
monitor_statistic [enable]
ex: monitor_statistic 1
    enable statistic monitor
        """
        [ret,pars] = self._line_set(line,["1"],"missing enable information, use default: 1")
        if pars[0]=="1":
            if not self.stat_sub:
                self.stat_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self.cbDiagnostics)
                rospy.loginfo("sys monitor statistic been setup!" )
        else:
            if self.stat_sub:
                self.stat_sub.unregister()
                self.stat_sub = None
    def cbDiagnostics(self,diag):
        rospy.loginfo("cli_node:receive from /diagnostics: %s" %(diag))

    def cbFeedback(self,feedback):
            rospy.loginfo("feedback: %s" %(feedback))


    def do_actionclient(self,line):
        """ all components action client example
actionclient [act_id] [act_subid] [i1] [i2] [f1] [f2] [line]
ex: actionclient 101 1 0 0 0.0 0.0 test
    setup harvest goal , subid=1
    current act_id valid start(100,300,400,500), range 100
        
        """
        [ret,pars] = self._line_set(line,["101","1", "0", "0", "0.0", "0.0", "test"],"missing enable information, use default: 101 1 0 0 0.0 0.0 test")

        act_id = int(pars[0])
        act_subid= int(pars[1])
        act_i1=int(pars[2])
        act_i2=int(pars[3])
        act_f1=float(pars[4])
        act_f2=float(pars[5])
        act_line=pars[6]
#        grp_name_def=["harvest","cli","vision","arm","car","pe","sim"]       
        consts= Consts()
        act_grp = consts.par_to_group(act_id)
        rospy.loginfo("act_grp=%s" %(act_grp))  # A FibonacciResult
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient('%s_act' %(act_grp), FhbActAction)
    
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
    
        # Creates a goal to send to the action server.
        goal = FhbActGoal()
        #a gold for debug purpose
        goal.act_id=act_id
        goal.act_subid=act_subid 
        goal.i1=act_i1
        goal.i2=act_i2
        goal.f1=act_f1
        goal.f2=act_f2
        goal.line=act_line
         
    
        # Sends the goal to the action server. assign feedback callback
        # send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None)
        client.send_goal(goal,None,None,self.cbFeedback)
    
        # Waits for the server to finish performing the action.
        client.wait_for_result()
    
        # Prints out the result of executing the action
        rospy.loginfo( client.get_result())  # A FibonacciResult

        

#CLI-Harvest level
class CliHarvest(RootCli):
        
    def cbFeedback(self,feedback):
        rospy.loginfo("feedback: %s" %(feedback))
            

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
    def __init__(self):
        cmd.Cmd.__init__(self)
        self.fsm_event_pub = rospy.Publisher('fsm_event', FSMEvent, queue_size=1)

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
    def do_bagtest_example(self,line):
        """test by bag file example"""
        #this test need to manual setup record duration, the duration need > bag play time + response time of expacted behavior
        #this example need ./big.bag 
        cmd_record="rosbag record --duration=17 -O result.bag -e '/harvest_act/result'"
        cmd_play="rosbag play big.bag --topics /harvest_act/goal"
        self.p_record = subprocess.Popen(cmd_record, stdin=subprocess.PIPE, shell=True, cwd=".")
        self.p_play = subprocess.Popen(cmd_play, stdin=subprocess.PIPE, shell=True, cwd=".")
        self.p_play.wait()
        self.p_record.wait()
        
        #check there are records in recorded file
        bag = rosbag.Bag('result.bag')
        msg_exist=False
        for topic, msg, t in bag.read_messages(topics=['/harvest_act/result']):
             rospy.loginfo("recorded msg=%s" %(msg))
             msg_exist=True
        bag.close()
        if msg_exist == False:
            rospy.loginfo("No msg recorded")
        
        #rospy.sleep(20)
        #self.p_record.send_signal(subprocess.signal.SIGINT)

    def do_fsm_event(self,line):
        """fsm test by manual publishing fsm tranisition events
fsm_event [event]
ex: fsm_event go_to_pos
    send transition event to change fsm state
    """
        [ret,pars] = self._line_set(line,["joystick_mode"],"missing event information, use default: joystick_mode")
        if self.fsm_event_pub:
            fsm_event = FSMEvent()
            fsm_event.header = std_msgs.msg.Header()
            fsm_event.header.stamp = rospy.Time.now()
            fsm_event.publisher = "cli"
            fsm_event.event = pars[0]
            self.fsm_event_pub.publish(fsm_event)
            rospy.sleep(1)

#  - rosbag record -O result.bag -e '/harvest_act/result'
#  - rosbag play big.bag --topics /harvest_act/goal

        
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

#Cli action server
class CliActSrv(FhbActSrv):

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FhbActAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.cmdfun = None
    def __del__(self):
        print "in CliActSrv del"
        self._as.__del__()
        del self._as
        self._as = None
    def assign_cmdfun(self,cmdfun):
        self.cmdfun = cmdfun
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
        if goal.act_id>=Consts.ACTID_START_CLI:
            if goal.act_id == Consts.ACTID_START_CLI+ACTID_CLI_CMD:
                askcmd = goal.line
                cmd_ret = self.cmdfun(askcmd)
        else:
            success = False

        if success:
            self._result.status = GoalStatus.SUCCEEDED #1 # 0-ok, >1 error code
            self._result.error_level =0 # 0-warning, 1-error, 2-fatal error 
            self._result.error_msg =cmd_ret 
            rospy.loginfo('%s Succeeded :%s' % (self._action_name,self._result.error_msg))
            self._as.set_succeeded(self._result)
        else:
            self.success_result("fail, not act_id not in valid range")


def main():
    
    rospy.init_node('base_cli')
    #cas = CliActSrv("cli_act")
    rospy.loginfo("----- %s V%s" %(PRJNAME,str(VERSION)))
    rospy.loginfo("----- CLI need to run with ros master exist!" )
    bcli = BaseCli()
    while(1):
        try:
            
            bcli.cmdloop()

            if bcli.user_quit:
                rospy.signal_shutdown("test")
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