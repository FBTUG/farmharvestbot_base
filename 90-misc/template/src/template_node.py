#!/usr/bin/env python
# Description: 
#    Detail: 
# Author: 
# Pubs: /topic ( msg ) # description
# Subs: /topic ( msg ) # description
# Services: /topic ( msg ) # description
# Test Hint:
#    rostopic pub -r 1 /cmd_vel geometry_msgs/Twist '{ linear : {  x: 0, y: 0, z: 0 }, angular: {  x: 0, y: 0, z: 0 } }'
#    rostopic echo /arm_ctrl_state


import rospy



# Class description
class TemplateNode():
    def __init__(self):
        pass



def main():
    pass
       

if __name__ == '__main__':
    main()