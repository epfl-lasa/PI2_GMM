#!/usr/bin/env python
import subprocess

import roslib
roslib.load_manifest('trajectoryPublisher')

import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
    

if __name__=='__main__':
    bUseSpaceMouse = False
    topics = subprocess.check_output(["rostopic","list"]).split()
    if '/spaceMouse/message' in topics:
        bUseSpaceMouse = True
    
    rospy.init_node('wrenchTestPub')
    pub = rospy.Publisher('/ForceSensor',WrenchStamped)
    rospy.loginfo('started fake force sensor')
    r = rospy.Rate(100)
    msg = WrenchStamped()
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        r.sleep()





