#!/usr/bin/env python
# Description: node of finite state machine (fsm) manager
#    features:
#        provide fsm configuration and transition functions
#    logic:
#        
# Author: aga3134@gmail.com 
# Pubs: /fsm_state (FSMState) #current fsm state
# Subs: /fsm_event (FSMEvent) #fsm tranisition event
# Actions:
# Statistic:
# Reference: 
# Test Hint:
#    rostopic pub -1 /fsm_event farmharvestbot_msgs/FSMEvent "{header: auto, publisher: test, event: go_to_pos}"
#    rostopic echo /fsm_state

import rospy
from farmharvestbot_msgs.msg import *
from fhb_utils import FhbNode

class FSMNode(FhbNode):
    def __init__(self):
        FhbNode.__init__(self,rospy.get_name())
        self.state_msg = FSMState()
        self.state_msg.publisher = self.name
        self.fsm_state_pub = rospy.Publisher('fsm_state', FSMState, queue_size=1) 
        rospy.Subscriber('fsm_event', FSMEvent, self.cbFSMEvent)
        self.setup()
        
    def setup(self):
        self.states_dict = rospy.get_param("~states",{})
        self.global_transitions_dict = rospy.get_param("~global_transitions", {})
        self.state_msg.state = rospy.get_param("~initial_state","")

    def cbFSMEvent(self,fsm_event):
        if self.state_msg.state not in self.states_dict:
            return rospy.logerr("current state not in state dict. Please make sure fsm.yaml is correct")

        trans = self.states_dict[self.state_msg.state]["transitions"]
        if not trans:
            return rospy.logerr("No transition in current state. Please make sure fsm.yaml is correct")
        
        if fsm_event.event in trans:
            self.state_msg.state = trans[fsm_event.event]
            rospy.loginfo("recieve event %s from %s, change to state %s" % (fsm_event.event,fsm_event.publisher,self.state_msg.state))
            self.fsm_state_pub.publish(self.state_msg)
        else:
            #check global transition
            if fsm_event.event in self.global_transitions_dict:
                self.state_msg.state = self.global_transitions_dict[fsm_event.event]
                rospy.loginfo("recieve global event %s from %s, change to state %s" % (fsm_event.event,fsm_event.publisher,self.state_msg.state))
                self.fsm_state_pub.publish(self.state_msg)
            else:
                rospy.logerr("Invalid transition event %s from %s" % (fsm_event.event, fsm_event.publisher))

    

# main process loop        
def main():
    rospy.init_node('fsm_node')
    rospy.loginfo("fsm_node started")

    fn = FSMNode()
    rospy.spin()
       

if __name__ == '__main__':
    main()