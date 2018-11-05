#!/usr/bin/env python
# Description: Manager of harvest device
#    features: 
# Author: wuulong@gmail.com

import rospy
from fhb_utils import FhbNode

# Arm base class
class Harvest(FhbNode):
    def __init__(self):
        pass
    def rpt_status(self):
        return "OK"
    def rpt_join_state(self):
        return None


def main():
    pass
       

if __name__ == '__main__':
    main()