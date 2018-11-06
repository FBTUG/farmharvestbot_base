#!/usr/bin/env python
# Description: fake arm device to simulate hardware from serial
#    direction from arduino device perspective
# Author: wuulong@gmail.com
# Pubs: 
#    /serial_tx ( std_msgs/String ) # console tx to PC
# Subs:
#    /serial_rx ( msg ) # accept console command from user
# Test Hint:
#    rostopic pub -r 1 /serial_rx std_msgs/String '{ data: R01 }'
#    rostopic echo /serial_tx
import rospy
from std_msgs.msg import String
import time

STATUS_OK = "R00"
STATUS_CMDDONE="R02"


# Fake arm device
class FakeArm():
    def __init__(self):
        self.serial_tx_pub = None #
        self.serial_rx_sub = None
        self.tx_cnt=0
        self.rx_cnt=0 
        
        
    def setup(self):
        #self.fba = arm.FarmBotArm() 
        self.serial_tx_pub = rospy.Publisher('serial_tx', String, queue_size=1)
        self.serial_rx_sub = rospy.Subscriber('serial_rx', String, self.cbSerialRx)

    def pubSerialTx(self,msg):
        self.serial_tx_pub.publish(msg)
        self.tx_cnt= self.tx_cnt + 1 
    def cbSerialRx(self,rx_str):
        rospy.loginfo("fake_arm:receive from serial: %s" %(rx_str))
        self.rx_cnt = self.rx_cnt + 1
        self.pubSerialTx(STATUS_CMDDONE)
        

# every 1 second, STATUS_OK
def main():
    rospy.init_node('fake_arm_node')
    rospy.loginfo("fake_arm_node started")

    fa = FakeArm()
    fa.setup()
    rate = rospy.Rate(10) # 10hz
    t_last = rospy.Time.from_sec(time.time())
    dur_period = rospy.Duration(1) #unit: second
    while not rospy.is_shutdown():
        t_cur = rospy.Time.from_sec(time.time()) 
        if (t_cur - t_last) >= dur_period:
            str_status = STATUS_OK
            #send OK every second
            fa.pubSerialTx(str_status)
            t_last = t_cur
        rate.sleep() 
       

if __name__ == '__main__':
    main()