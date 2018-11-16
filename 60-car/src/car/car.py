#!/usr/bin/env python
# Description: car device
#    features: 
# Author: wuulong@gmail.com

import rospy
from motor_driver import MotorDriver

# Car  class
class CarBase(object):
    def __init__(self):
        self._gain=0.1 # float 0-1
        self._tune=0   # float left/right
        self._style=0  # control wheel number: 2-wheel=2, 1-wheel=1
        self._wheel_dist=100 # wheel distance between control wheel, unit: mm
        self._speed_level=0 # [ high_speed=3, middle_speed=2, low_speed=1]
        self._speed_to_gain={1:0.3,2:0.6,3:1.0}
        self.m_drv= None#MotorDriver()
    #control
    def go_front(self,control_rate):
        pass
    def go_back(self,control_rate):
        pass
    def go_pos(self,diff_pos):
        pass
    def turn_right(self,pos_move):
        pass
    def turn_left(self,pos_move):
        pass
    def turn_degree(self,diff_degree):
        pass
    def set_speed(self,speed_level):
        self._speed_level=speed_level
        self.gain = self._speed_to_gain[speed_level]
        pass
    def set_gain(self,gain_value):
        self._gain = gain_value
        pass
    def set_turn(self,turn_value):
        self._turn = turn_value
        pass
    def emergency_stop(self):
        pass
    def set_normal(self):
        pass
    def set_pos_cur(self):
        pass
    #status
    def rpt_status(self):
        return "OK"
    def rpt_join_state(self):
        return None

class Car(CarBase):  
    def __init__(self):
        super(Car, self).__init__()
        self.m_drv= MotorDriver()
         
