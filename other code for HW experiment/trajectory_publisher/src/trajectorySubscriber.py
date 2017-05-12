#!/usr/bin/env python

import os
import time
from collections import deque
import cPickle as pickle

import roslib
roslib.load_manifest('trajectory_publisher')
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped,WrenchStamped
from std_msgs.msg import String

from messageSync import syncAndDownSample, getMsgTime

FORCE_TOPIC_NAME = '/comp_wrenchBase_pub'
POSE_TOPIC_NAME = 'KUKA/Pose'
STATUS_TOPIC_NAME = 'KUKA/TaskStatus'
RECORDING_FREQ = 20

def formatLine(poseMsg,wrenchMsg,t=None,zeroTimeMsg=None):
    if t==None and zeroTimeMsg==None:
        t = poseMsg.header.stamp.to_sec()
    elif t==None:
        t = getMsgTime(zeroTimeMsg,poseMsg)
    line = str(t)+' '
    line = line+str(poseMsg.pose.position.x)+' '
    line = line+str(poseMsg.pose.position.y)+' '
    line = line+str(poseMsg.pose.position.z)+' '
    
    line = line+str(poseMsg.pose.orientation.x)+' '
    line = line+str(poseMsg.pose.orientation.y)+' '
    line = line+str(poseMsg.pose.orientation.z)+' '
    line = line+str(poseMsg.pose.orientation.w)+' '

    line = line+str(wrenchMsg.wrench.force.x)+' '
    line = line+str(wrenchMsg.wrench.force.y)+' '
    line = line+str(wrenchMsg.wrench.force.z)+' '

    line = line+str(wrenchMsg.wrench.torque.x)+' '
    line = line+str(wrenchMsg.wrench.torque.y)+' '
    line = line+str(wrenchMsg.wrench.torque.z)
    return line




class trajectorySubscriber():
    def __init__(self):
        self.recording = False
        # initialize subscriber
        rospy.init_node('trajectorySubscriber')
        rospy.Subscriber(POSE_TOPIC_NAME,PoseStamped,self.newPoseCallback)
        rospy.Subscriber(FORCE_TOPIC_NAME,WrenchStamped,self.newWrenchCallback)
        rospy.Subscriber(STATUS_TOPIC_NAME,String,self.newStatusCallback)
        # folder for storing the received data
        # get path to trajectory_publisher ros package
        p = rospkg.RosPack().get_path('trajectory_publisher')
        self.p_store = p+'/recordings/'
        fs = os.listdir(self.p_store)
        self.recNb = len(fs)+1
        self.posedeque = deque()
        self.wrenchdeque = deque()
        self.status = None
        rospy.loginfo("starting node")
        
    def newPoseCallback(self,newPose):
        if(self.recording):
            self.posedeque.append(newPose)

    def newWrenchCallback(self,newWrench):
        if(self.recording):
            self.wrenchdeque.append(newWrench)

    def newStatusCallback(self,newStatus):
        self.status = newStatus.data
        

    def run(self):
        while not rospy.is_shutdown():
            if (not self.recording) and self.status=='EXECUTING':
                self.startRecording()

            if self.recording and self.status != 'EXECUTING':
                self.stopRecording()
            time.sleep(0.1)
            
    def startRecording(self):
        rospy.loginfo("Started recording.")
        self.recording = True

    def stopRecording(self):
        self.stopRecordingNew()

    def stopRecordingNew(self):
        self.recording = False
        rospy.loginfo("Stopping recording..")
	print len(self.posedeque)
	print len(self.wrenchdeque)
        sMsgList = syncAndDownSample([self.posedeque,self.wrenchdeque],RECORDING_FREQ)
        if sMsgList[0][0]._type =="geometry_msgs/PoseStamped":
            poselist_s = sMsgList[0]
            wrenchlist_s = sMsgList[1]
        elif sMsgList[0][0]._type =="geometry_msgs/WrenchStamped":
            poselist_s = sMsgList[1]
            wrenchlist_s = sMsgList[0]
        lines = list()
        for k in range(0,len(poselist_s)):
            line = formatLine(poselist_s[k], wrenchlist_s[k],zeroTimeMsg=sMsgList[0][0])
            lines.append(line)
            k+=1
        # open a file for putting the recorded data
        f = open(self.p_store+'rec'+str(2),'w')
        # f = open(self.p_store+'rec'+str(self.recNb),'w')
        # write it
        f.writelines("\n".join(lines))
        f.close()
        rawData = {'poses': self.posedeque, 'wrenches': self.wrenchdeque}
        fRaw = open(self.p_store+'/raw/'+'rec'+str(self.recNb),'wb')
        pickle.dump(rawData,fRaw)
        fRaw.close()
        self.recNb+=1
        self.posedeque.clear()
        self.wrenchdeque.clear()
        rospy.loginfo("Saved recorded data.")
        

    def stopRecordingOld(self):
        rospy.loginfo("stopped recording")
        self.recording = False
        T = 1.0/RECORDING_FREQ
        poseMsg = self.posedeque.popleft()
        wrenchMsg = self.wrenchdeque.popleft()
        zMsg = poseMsg
        t = 0.0;
        line = formatLine(poseMsg,wrenchMsg,t)
        lines = list()
        lines.append(line)
        save_msg_pose = 0
        save_msg_wrench = 0
        
        while True:
            # keep track of time stamps of last written messages
            pose_ot = poseMsg
            wrench_ot = wrenchMsg
            save_msg_pose +=1
            save_msg_wrench +=1

            while True:
                poseMsg = self.posedeque.popleft()
                if(len(self.posedeque)<1):
                    break
                prevDiffError = abs(getMsgTime(pose_ot,poseMsg)-T)
                # check if popping another message leads to shorter time difference error
                diffError = abs(getMsgTime(pose_ot,self.posedeque[0])-T)
                if(diffError>prevDiffError):
                    # if it does not, stop popping and save current msg
                    break

            while True:
                wrenchMsg = self.wrenchdeque.popleft()
                if(len(self.wrenchdeque)<1):
                    break
                prevDiffError = abs(getMsgTime(wrench_ot,wrenchMsg)-T)
                # check if popping another message leads to shorter time difference error
                diffError = abs(getMsgTime(wrench_ot,self.wrenchdeque[0])-T)
                if(diffError>prevDiffError):
                    # if it does not, stop popping and save current msg
                    break

            # update time
#            t = t+T
            t=getMsgTime(zMsg,poseMsg)
            # add a formatted line from the messages to the list of lines
            lines.append(formatLine(poseMsg,wrenchMsg,t))
            # if there are no more messages we are done
            if len(self.posedeque)<1 or len(self.wrenchdeque)<1:
                break
        # open a file for putting the recorded data
        # f = open(self.p_store+'rec'+str(self.recNb),'w')
	f = open(self.p_store+'rec'+str(2),'w')	
        # write it
        f.writelines("\n".join(lines))
        f.close()
        self.recNb+=1
        self.posedeque.clear()
        self.wrenchdeque.clear()

if __name__=='__main__':
    ts = trajectorySubscriber()
    ts.run()
        
    

    




