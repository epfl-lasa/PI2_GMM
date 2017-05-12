


/*
  A simple tool for generating a smooth orientation-trajectory that will take you from one orientation to another using spherical linear interpolation.

  initialize like this:

    OrientationInterpolator myOrientInterp;

    myOrientInterp.SetRobot(mRobot);

    Matrix3 desiredTargetOrientation;

 assign to this matrix your desired final orientation, and then send it to the interpolator:

    myOrientInterp.SetTarget(desiredTargetOrientation);

by default, the orientation is assumed to refer to the link "TOOLPLATE" in the robot kinematic chain. If your tool is called twoHandedSword, you can do:
    myOreintInterp.SetToolName("twoHandedSword");



 use case 1: time dependent interpolation

    myOrient.SetInterpolationTime(duration, dt);

sets the current orientation as the starting orientation for the interpolation

    myOrient.Reset();

in your control loop, do this:

find the compute the current interpolated orientation

  myOrient.UpdateTimeDependent();

get the result as a rotation matrix (Mathlib Matrix3)
  myOrient.GetOutput(resultMatrix3);
or get the result  in angle-axis represnation (Mathlib Vector3)
  myOrient.GetOutput(resultVector3);





 use case 2: time independent interpolation

    myOrient.Reset();

in your control loop, do this:

find the compute the current interpolated orientation

  myOrient.Update(interp);

here, interp is a scalar in the interval [0,1] which you may compute however you like. It can e.g. be a fucntion that smoothly
goes from 0 to one when the robot is approaching a point in the task space etc.

get the result as a rotation matrix (Mathlib Matrix3)
  myOrient.GetOutput(resultMatrix3);
or get the result  in angle-axis represnation (Mathlib Vector3)
  myOrient.GetOutput(resultVector3);




  */


#ifndef ORIENTATIONINTERPOLATOR_H
#define ORIENTATIONINTERPOLATOR_H

#include "RobotLib/Robot.h"
#include "MathLib/MathLib.h"
#include "MathLib/Matrix3.h"

#include "eigen3/Eigen/Dense"


class OrientationInterpolator
{

    Robot * mRobot;

    //Eigen::Map<Eigen::Matrix3d> * targetOrientationAsEigen;

    Eigen::Quaternion<double> * qTarget;
    Eigen::Quaternion<double> * qOutput;
    Eigen::Quaternion<double> * qStart;
    Eigen::Quaternion<double> * qDelta;
    Eigen::Quaternion<double> * qLast;


    Matrix3 targetOrientationAsMatrix3;
    Matrix3 startOrientationAsMatrix3;
    Matrix3 outputOrientationAsMatrix3;
    Vector3 outputOrientationAsAngleAxis;

    Eigen::Matrix3d startOrientationAsEigen;
    Eigen::Matrix3d targetOrientationAsEigen;
    Eigen::Matrix3d outputOrientationAsEigen;

    string toolName;

    double interpolationTime;
    double dT;
    bool bTimeDep;
    double currTime;


    double interpolationParam;



public:
    OrientationInterpolator();

    // giving a pointer to the robot object
    void SetRobot(pRobot thisRobot);
    // set the name of the tool for which you intend to control the orientation
    void SetToolName(string tname);
    // start the interpolation: will save the current orientation as a starting point for the interpolation
    void Reset();
    // set how long the transition between current and target orientation should take
    void SetInterpolationTime(REALTYPE interpT, REALTYPE dt);
    // set the target orientation, rotation matrix representation
    void SetTargetOrientation(const Matrix3 & targetOrient);
    //manually specify the interpolation parameter (e.g. for time-independent orientation generation), interp \in [0,1]
    //void SetInterpolationParameter(REALTYPE interp);
    //get the output in rotation matrix representation
    void GetOutput(Matrix3 & outputRotMat);
    //get the output in angle-axis representation
    void GetOutput(Vector3 & outputAngleAxis);
    // update with time dependency
    void UpdateTimeDependent();
    //update time independent, you need to set interpolation paramter for anything to happen.
    void Update(double interp);
    // update with a specified angular velocity and time step
    void UpdateWithAngularVelocity(Vector3 omega,double dt);
    // auxiliary function, not to be called manually
    void ConvertOutput();



};

#endif // ORIENTATIONINTERPOLATOR_H
