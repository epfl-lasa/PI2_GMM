#!/usr/bin/env python

import roslib
roslib.load_manifest('trajectoryPublisher')

import numpy as np
import rospy
import os
import rospkg
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Header
from collections import deque

POSE_TOPIC_NAME = 'KUKA/Pose'
STATUS_TOPIC_NAME = 'KUKA/TaskStatus'
RECORDING_FREQ = 500

def formatLine(t,poseMsg):
    # line = str(t)+' '
    # line = line+str(poseMsg.pose.position.x)+' '
    
    line = str(poseMsg.pose.position.x)+' '
    line = line+str(poseMsg.pose.position.y)+' '
    line = line+str(poseMsg.pose.position.z)+' '
    
    line = line+str(poseMsg.pose.orientation.x)+' '
    line = line+str(poseMsg.pose.orientation.y)+' '
    line = line+str(poseMsg.pose.orientation.z)+' '
    line = line+str(poseMsg.pose.orientation.w)+' '

    return line


def getMsgTime(zeroMsg,nowMsg):
    zeroHeader = zeroMsg.header
    msgHeader = nowMsg.header
    zeroTime = zeroHeader.stamp.secs+zeroHeader.stamp.nsecs*1e-9
    msgTime = msgHeader.stamp.secs+msgHeader.stamp.nsecs*1e-9
    return msgTime-zeroTime;
    


class simpleRecord():
    def __init__(self):
        self.recording = False
        # initialize subscriber
        rospy.init_node('simpleRecord')
        rospy.Subscriber(POSE_TOPIC_NAME,PoseStamped,self.newPoseCallback)
#        rospy.Subscriber(FORCE_TOPIC_NAME,WrenchStamped,self.newWrenchCallback)
        rospy.Subscriber(STATUS_TOPIC_NAME,String,self.newStatusCallback)
        # folder for storing the received data
        # get path to trajectoryPublisher ros package
        p = rospkg.RosPack().get_path('trajectoryPublisher')
        self.p_store = p+'/recordings/'
        fs = os.listdir(self.p_store)
        self.recNb = len(fs)+1
        self.posedeque = deque()
        self.status = None
        
    def newPoseCallback(self,newPose):
        if(self.recording):
            self.posedeque.append(newPose)

    def newStatusCallback(self,newStatus):
        self.status = newStatus

    def run(self):
        cnt = 0
        wait_time =10
        rec_time=15
        while True:
            time.sleep(1)
            print wait_time-cnt
            cnt+=1
            if wait_time-cnt<0:
                break;
        self.startRecording()
        cnt = 0
        while True:
            time.sleep(1)
            print rec_time-cnt
            cnt+=1
            if rec_time-cnt<0:
                break;
        self.stopRecording()
            
    def startRecording(self):
        self.recording = True

    def stopRecording(self):
        self.recording = False
        T = 1.0/RECORDING_FREQ
        poseMsg = self.posedeque.popleft()
        t = 0.0;
        line = formatLine(t,poseMsg)
        lines=list()
        lines.append(line)
        zMsg = poseMsg
        received_msg = len(self.posedeque)
        save_msg = 0
        while True:
            # keep track of time stamps of last written messages
            pose_ot = poseMsg
            save_msg += 1
            # downsample and sync messages
            while True:
                poseMsg = self.posedeque.popleft()
                # abort imediately if there are no more messages
                if(len(self.posedeque)<1):
                    break
                prevDiffError = abs(getMsgTime(pose_ot,poseMsg)-T)
                # check if popping another message leads to shorter time difference error
                diffError = abs(getMsgTime(pose_ot,self.posedeque[0])-T)
                if(diffError>prevDiffError):
                    # if it does not, stop popping and save current msg
                    break
            t=getMsgTime(zMsg,poseMsg)
            # add a formatted line from the messages to the list of lines
            lines.append(formatLine(t,poseMsg))
            # if there are no more messages we are done
            if len(self.posedeque)<1:
                break
        # open a file for putting the recorded data
        print 'received msgs: ', received_msg
        print 'discarded msgs: ', received_msg-save_msg
        print 'saved msgs: ', save_msg
        f = open(self.p_store+'rec'+str(self.recNb),'w')
        # write it
        f.writelines("\n".join(lines))
        f.close()
        self.recNb+=1
        self.posedeque.clear()

if __name__=='__main__':
    ts = simpleRecord()
    ts.run()
        
    

    




