import rospy
import rospkg
import rosbag
import actionlib
from actionlib_msgs.msg import GoalStatus
from farmharvestbot_msgs.msg import *
from fhb_utils import Consts
import diagnostic_updater
import diagnostic_msgs
import subprocess, os, signal

#misc utility functions
class FhbMisc:
    def __init__(self):
        pass
    def bag_play_record(self,cmd_record,cmd_play, b_wait):
        """play bag, record another and wait
        """
        self.p_record = subprocess.Popen(cmd_record, stdin=subprocess.PIPE, shell=True, cwd=".")
        self.p_play = subprocess.Popen(cmd_play, stdin=subprocess.PIPE, shell=True, cwd=".")
        self.p_play.wait()
        self.p_record.wait()

    def get_package_dir(self,package_name):
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path(package_name)
        return pkg_path
    def bag_have_msg_from_topic(self,bagfile, topic,b_show=False):
        msg_exist=False
        bag = rosbag.Bag(bagfile)
        for topic, msg, t in bag.read_messages(topics=[topic]):
            if b_show:
                rospy.loginfo("recorded msg=%s" %(msg))
            msg_exist=True
        bag.close()
        if msg_exist == False:
            if b_show:
                rospy.loginfo("No msg recorded")
        return msg_exist
#all statistic suggest to have this parent
class FhbStatistic:
    def __init__(self,name):
        self.fruit_cnt=0
        self.name = name
        self.setup()
        self.local_init()

    def setup(self):
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID(self.name)
        self.updater.add("%s_statistic" %(self.name), self.produce_statistic)
    def local_init(self):
        pass
    def produce_statistic(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "%s statistic" %(self.name))
        self.stat_add(stat)
        return stat
    #replace this
    def stat_add(self,stat):
        stat.add("fruit_cnt", self.fruit_cnt)
        self.fruit_cnt= self.fruit_cnt+1


#FHB action server
class FhbActSrv(object):
    # create messages that are used to publish feedback/result
    _feedback = FhbActFeedback()
    _result = FhbActResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FhbActAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    def success_result(self,error_msg):
        self._result.status = GoalStatus.SUCCEEDED #1 # 0-ok, >1 error code
        self._result.error_level =0 # 0-warning, 1-error, 2-fatal error 
        self._result.error_msg =error_msg
        
        rospy.loginfo('%s Succeeded :%s' % (self._action_name,error_msg))
        
        self._as.set_succeeded(self._result)

    def fail_result(self,error_msg):
        self._result.status = GoalStatus.REJECTED #1 # 0-ok, >1 error code
        self._result.error_level =1 # 0-warning, 1-error, 2-fatal error 
        self._result.error_msg =error_msg
        rospy.loginfo('%s Fail :%s' % (self._action_name,error_msg))   
        self._as.set_succeeded(self._result)        
              
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # setup feedback msg
        self._feedback.notice_id = 0
        self._feedback.progrsss=10
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing... Goal =\n%s' % (self._action_name,  goal))
        
        # start executing the action
        if goal.act_id>=Consts.ACTID_START_HARVEST:
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

#all nodes suggest to have this parent
class FhbNode():
    def __init__(self, name):
        pass