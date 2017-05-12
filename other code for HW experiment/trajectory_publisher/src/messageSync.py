#!/usr/bin/env python


import roslib
roslib.load_manifest('trajectory_publisher')

import time
import numpy as np
import rospy
import os
import rospkg
from geometry_msgs.msg import PoseStamped


def getMsgTime(zeroMsg,nowMsg):
    zeroHeader = zeroMsg.header
    msgHeader = nowMsg.header
    zeroTime = zeroHeader.stamp.secs+zeroHeader.stamp.nsecs*1e-9
    msgTime = msgHeader.stamp.secs+msgHeader.stamp.nsecs*1e-9
    return msgTime-zeroTime;

def getKey(item):
    return item[0]

def sortListOfLists(listMsgArray):
    # will sort the list of len_objects in order of increasing length of len_object. len_object can be anything that supports the len(len_object) command (e.g. lists, deques, ..)
    listMsgArrayLen = list()
    # make a list of tuples, each of which has the msgArray and the length of the msgArray
    listMsgArrayLen = [(len(msgArray),msgArray) for msgArray in listMsgArray]
    # sort this new list of tuples according to the length of each message arry
    listMsgArrayLen = sorted(listMsgArrayLen,key=getKey)
    # create a new list with just the sorted arrays 
    listMsgArrayLen = [tup[1] for tup in listMsgArrayLen]
    return listMsgArrayLen



def syncAndDownSample(raw_listMsgArray,freq=100):
    listMsgArray = sortListOfLists(raw_listMsgArray)

    T = 1.0/float(freq)
    msgArray = listMsgArray[0]
    k=0
    # downsample slowest message stream first
    while len(msgArray)<=k+2:
        diffError = abs(getMsgTime(msgArray[k],msgArray[k+1])-T)
        nextDiffError = abs(getMsgTime(msgArray[k],msgArray[k+2])-T)
        if diffError > nextDiffError:
            del(msgArray[k+1])
        else:
            k += 1

    baseMsgArray = msgArray
    retListMsgArray = [baseMsgArray]
    for msgArray in listMsgArray[1:]:
        mp = iter(msgArray)
        mpn = iter(msgArray)
        mpn.next()
        newMsgArray = list()
        for msg in baseMsgArray:
            while True:
                try:
                    thisMsg = mp.next()
                except StopIteration:
                    break
                try:
                    nextMsg = mpn.next()
                except StopIteration:
                    break
                thisError = abs(getMsgTime(msg,thisMsg))
                nextError = abs(getMsgTime(msg,nextMsg))
                if(thisError <= nextError):
                    break
            newMsgArray.append(thisMsg)
        retListMsgArray.append(newMsgArray)
            
            

    # #    listMsgArray[0] = msgArray
    # retListMsgArray = [msgArray for ma in listMsgArray]
    # # for each message in the downsampled array, find closest match in other message streams
    # p=0
    # for msg in retListMsgArray[0]:
    #     k=0
    #     for msgArray in listMsgArray[1:]:
    #         j=0
    #         print k,j
    #         for msg2 in msgArray:
    #             diffError = abs(getMsgTime(msg,msgArray[j]))
    #             nextDiffError = abs(getMsgTime(msg,msgArray[j+1]))
    #             if(nextDiffError>diffError):
    #                 break
    #             else:
    #                 j += 1
    #             if j+1>=len(msgArray):
    #                 break
                
    #         retListMsgArray[k][p]=msgArray[j]
    #         k+=1
    #     p+=1

    
                
                
    return retListMsgArray
    
        
            
        

def simple_test_function():
        # create some message arrays
    l=[PoseStamped() for i in range(0,100)]
    l2=[PoseStamped() for i in range(0,200)]
    l3=[PoseStamped() for i in range(0,300)]

    tt = 0.0
    dt = 1.0/400.0
    for msg in l:
        t = rospy.Time.from_sec(tt)
        msg.header.stamp = t# rospy.Time.now()
        tt += dt
        #print msg.header.stamp.to_sec()

    tt = 0.0
    dt = 1.0/800.0
    for msg in l2:
        t = rospy.Time.from_sec(tt)
        msg.header.stamp = t# rospy.Time.now()
        tt += dt
        #print msg.header.stamp.to_sec()

    tt = 0.0
    dt = 1.0/1200.0
    for msg in l3:
        t = rospy.Time.from_sec(tt)
        msg.header.stamp = t# rospy.Time.now()
        tt += dt
        #print msg.header.stamp.to_sec()




    ll = [l,l2,l3]
    
    lr = syncAndDownSample(ll)
    for l in lr:
        print len(l)
    for msg in lr[0]:
        print msg.header.stamp.to_sec()
    # for msg in ll[1]:
    #     print msg.header.stamp.to_sec()
        
    
    

if __name__=='__main__':
    simple_test_function()
