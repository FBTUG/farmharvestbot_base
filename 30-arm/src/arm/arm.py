#!/usr/bin/env python
# Description: Manager of arm device
#    features: 
# Author: 

import rospy

ARM_CONTROLLER_TYPE_DEF=0
ARM_CONTROLLER_TYPE_GCODE=1
# Arm base class
class Arm():
    def __init__(self):
        self.link_cnt = 3 # default 3-Axis
        self.link_par = {} # link parameters definition
        self.controller_type = ARM_CONTROLLER_TYPE_GCODE # controller type
    def rpt_status(self):
        return "OK"
    def rpt_join_state(self):
        return None
# Xyz type arm
class XyzArm(Arm):
    def __init__(self):
        pass

# FarmBot arm
class FarmBotArm(XyzArm):
    def __init__(self):
        pass
    def do_gcode(self,gcode_cmd):
        pass


def main():
    pass
       

if __name__ == '__main__':
    main()