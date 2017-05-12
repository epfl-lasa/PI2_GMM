#ifndef DS_MOTION_GENERATOR_H
#define DS_MOTION_GENERATOR_H

#include "MathLib/MathLib.h"
#include "RobotLib/Robot.h"
#include "RobotLib/RobotTools.h"
#include "RobotLib/KinematicChain.h"
#include "GMR_L.h"



#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

  class DSMotionGenerator {

  public :
    DSMotionGenerator();
    
    void SetRobot(Robot* pRob);
    void LoadModel(const char*);
    void Update(REALTYPE dt);
    void Restart();
    void GetOutput(Vector & result);
    REALTYPE GetDirectionOfMotion(Vector & result);
    REALTYPE GetDirectionOfMotion(Vector & result,Vector & Orth1, Vector & Orth2);
    void SetAttractorPosition(const Vector & Target);
    void Resize();

    void SetMotionSpeed(REALTYPE speed);
    void SetMotionSpeed(bool b);

    void SetMaxSpeed(REALTYPE c);

  protected :
    Robot* pRobot;
    KinematicChain KinChain;
    RevoluteJointSensorGroup Encoders;

    bool bHaveRobot,bRestart;

    int nMode;
    enum{CART_SPACE,JOINT_SPACE};
    
    REALTYPE RunningTime;
    REALTYPE MotionSpeed;
    bool bUseExternalSpeed;

    REALTYPE cap;

    //    Vector3 CurrPos, AttractorPos,CurrPosRel;
    Vector3 tempV3_1,tempV3_2,tempV3_3;
    Vector CurrPosV,AttractorPosV,DesVelV,DesPosV,CurrPosRelV;
    GaussianMixture DSmodel;
  };

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
