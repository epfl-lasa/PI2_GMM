#ifndef SIMPLE_MOTION_GENERATOR_H
#define SIMPLE_MOTION_GENERATOR_H

#include "MathLib/MathLib.h"
#include "RobotLib/Robot.h"
#include "RobotLib/RobotTools.h"
#include "RobotLib/KinematicChain.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

  class SimpleMotionGenerator {

  public :
    SimpleMotionGenerator();
    
    void SetRobot(Robot* pRob);
    void Restart();
    void Update(REALTYPE dt);
    void GetOutput(Vector &);
    void SetTarget(Vector & Target);
    void Resize();
    void SetCap(REALTYPE);
    void SetMaxSpeed(Vector) ;
    void SetMaxSpeed(REALTYPE);
    void SetRiseTime(REALTYPE);
    void GetVelocity(Vector &);





  protected :
    Robot* pRobot;
    KinematicChain KinChain;
    RevoluteJointSensorGroup Encoders;
    
    bool bHaveRobot,bRestart;
    
    REALTYPE RunningTime;
    REALTYPE RiseTime;

    Vector mJointTarget,mJointVelocities,mCurrentJointPos;
    Vector MaxSpeeds;

    REALTYPE cap;
  };

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
