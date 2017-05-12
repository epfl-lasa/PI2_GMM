/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef KUKA_6dTRAJ_H_
#define KUKA_6dTRAJ_H_

#include "RobotLib/RobotInterface.h"
#include "KUKARobotModel/LWRRobot.h"
//#include <geometry_msgs/PoseArray.h>
#include "imp_messages_package/ImpedanceControl.h"
#include "imp_messages_package/ImpedanceControlArray.h"
#include <geometry_msgs/Quaternion.h>
#include "ros/ros.h"
#include "MotionGenerators/CDDynamics.h"
#include "GLTools/GLTools.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
//#include "LBCStiffnessAdaptation/VectorStamped.h"

#include <eigen3/Eigen/Dense>
#include <queue>


#define EXECUTE_HEIGHT_OFFSET 0.00


//typedef geometry_msgs::PoseArray TRAJECTORY_MSG;
//typedef  geometry_msgs::Pose POSE_MSG;
typedef imp_messages_package::ImpedanceControlArray TRAJECTORY_MSG;
typedef imp_messages_package::ImpedanceControl POSE_MSG;

typedef Eigen::Quaternion<REALTYPE> quat;

enum stateEnumerator {PREPARE,EXECUTE,GRAVCOMP,IDLE};


//std_msgs::String PrepStr;
//PrepStr.data = 'PREPARING';


class KUKA_6dTRAJ : public RobotInterface
{

  LWRRobot * mLWRRobot;
  Vector3 currPos;
  Matrix3 currOrient;

  ros::NodeHandle * rosNode;
  ros::Subscriber trajSub;
  //ros::Subscriber stiffnessSub;
  ros::Subscriber wrenchSub;

  stateEnumerator taskState;
  bool bTrajectoryFollowingEnabled;

  CDDynamics * cartesianPlanner;
  // a few quaternion objects to keep track of orientation interpolation
  quat  qTarget;
  quat  qStart;
  quat  qInterp;
  quat  qControl;
  Eigen::Matrix3d qControlMatrix;
  geometry_msgs::Quaternion * qTempRos;
  geometry_msgs::Point * pTempRos;
  double * kTempRos;

  REALTYPE init_dist,curr_dist,orient_interp;

  ros::Publisher statusPublisher;
  ros::Publisher cartesianErrorPublihser;
  ros::NodeHandle * nh;


  queue<TRAJECTORY_MSG> trajectoryQueue;
  uint currRow;
  Matrix * currTraj;
  Vector cartTarget;
  Vector cartCommandV;
  Vector3 cartCommand;
  Matrix3 cartCommandOrient;
  bool  bFirst;

  Vector gravCompStiffness;
  Vector movingStiffness;
  Vector RLStiffness;
  Vector minStiffness;

  Vector3 latestForceMeasurement;

  std_msgs::String ExecStr;
  std_msgs::String PrepStr;
  std_msgs::String IdleStr;


public:
            KUKA_6dTRAJ();
    virtual ~KUKA_6dTRAJ();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

    void referenceTrajectoryCallback(const TRAJECTORY_MSG::ConstPtr & Msg);
    void measuredForceCallback(const geometry_msgs::WrenchStamped wrenchMsg);
    //void stiffnessCallback(const LBCStiffnessAdaptation::VectorStamped::ConstPtr & Msg);

    void switchState(stateEnumerator);
    void RobotDraw();
    void PublishCartesianError();
};





#endif 
