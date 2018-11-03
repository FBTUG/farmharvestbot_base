import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from farmharvestbot_msgs.msg import *
from fhb_utils import Consts
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
        rospy.loginfo('%s: Executing...' % (self._action_name))
        
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
