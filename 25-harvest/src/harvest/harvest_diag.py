#!/usr/bin/env python
import rospy
import diagnostic_updater
import diagnostic_msgs

class HarvestStatistic:
    def __init__(self):
        self.fruit_cnt=0
    
    def produce_statistic(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK , "Harvest statistic")
        stat.add("fruit_cnt", self.fruit_cnt)
        self.fruit_cnt= self.fruit_cnt+1
        return stat

if __name__=='__main__':
    rospy.init_node("harvest_example")

    updater = diagnostic_updater.Updater()

    updater.setHardwareID("harvest")

    hs = HarvestStatistic()
    updater.add("harvest_statistic", hs.produce_statistic)

    while not rospy.is_shutdown():
        rospy.sleep(3)
        # We can call updater.update whenever is convenient. It will take care
        # of rate-limiting the updates.
        updater.update()

