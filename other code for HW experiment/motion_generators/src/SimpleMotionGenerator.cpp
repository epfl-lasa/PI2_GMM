
#include "SimpleMotionGenerator.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

SimpleMotionGenerator::SimpleMotionGenerator(){
  pRobot=NULL;

  bHaveRobot = false;
  cap = 1.0;
  RiseTime = 1.0;		//default rise time = 1 sec
  MaxSpeeds.Resize(7);
}

void SimpleMotionGenerator::Resize(){
  int nJoints = pRobot->GetDOFCount();
  mJointTarget.Resize(nJoints);
  mJointVelocities.Resize(nJoints);
  mCurrentJointPos.Resize(nJoints);
  
}

void SimpleMotionGenerator::Restart(){
  bRestart=true;
}

void SimpleMotionGenerator::SetCap(REALTYPE newCap){
  cap=newCap;
}

void SimpleMotionGenerator::SetRiseTime(REALTYPE _risetime){
    RiseTime = _risetime;
}

void SimpleMotionGenerator::GetVelocity(Vector & vel)
{
    vel = mJointVelocities;
}
void SimpleMotionGenerator::SetMaxSpeed(Vector newSpeed){
  if(newSpeed.Size() != MaxSpeeds.Size()){
    cout<<"specified max speed vector does not have the correct dimensionality!"<<endl;
    return;
  }
  MaxSpeeds = newSpeed;
}

void SimpleMotionGenerator::SetMaxSpeed(REALTYPE newSpeed){
  MaxSpeeds.One();
  MaxSpeeds *= newSpeed;
}


void SimpleMotionGenerator::Update(REALTYPE Dt){
  KinChain.Update();
  Encoders.ReadSensors();

  if(bRestart){
    mCurrentJointPos = Encoders.GetJointAngles();
    RunningTime = 0;
    bRestart = false;
  }
  mJointVelocities = mJointTarget;
  // cout<<"MG target"<<endl;
  // mJointTarget.Print();
  // cout<<"MG currpos"<<endl;
  // mCurrentJointPos.Print();
  mJointVelocities -= mCurrentJointPos;
  //  mJointVelocities.Print();

  // cout<<"MG vel"<<endl;
  // mJointVelocities.Print();

  // cout<<"MG vel"<<endl;
  // mJointVelocities.Print();
  //  mJointVelocities/=Dt;
  //  mJointVelocities*=0.001;
  //cap speed


  if(mJointVelocities.Norm()>cap){
    mJointVelocities.Normalize();
    mJointVelocities*=cap;
  }
  //also check if any of the individual joints move to fast..
  for(unsigned int i=0;i<mJointVelocities.Size();i++){
    if(fabs(mJointVelocities(i))>MaxSpeeds(i)){
      mJointVelocities(i) /= fabs(mJointVelocities(i));
      mJointVelocities(i) *= MaxSpeeds(i);
    }
  }


  //if movement just started, take it easy in the beginnning

  if(RunningTime<RiseTime){
    //    cout<<(sin(-PI/2+RunningTime/RiseTime*PI)+1)/2<<endl;
    mJointVelocities*=(sin(-PI/2+RunningTime/RiseTime*PI)+1)/2;
  }
  RunningTime+=Dt;
  //integrate 
  mJointVelocities.ScaleAddTo(Dt,mCurrentJointPos);
  // cout<<Dt<<endl;
  // cout<<"MG currpos"<<endl;
  // mCurrentJointPos.Print();

}

void SimpleMotionGenerator::SetTarget(Vector & target){
  if(target.Size()==mJointTarget.Size())
    mJointTarget = target;
  else{
    cout<<"MOTION GENERATOR: dimension mismatch when setting target. Resizing!"<<endl;
    mJointTarget.Print();
    target.Print();
    mJointTarget.Resize(target.Size());
    mJointTarget = target;
  }

}


void SimpleMotionGenerator::SetRobot(Robot* pRob){
  pRobot=pRob;
  bHaveRobot=true;
  if(pRobot==NULL)
    bHaveRobot=false;
  if(bHaveRobot){
    KinChain.SetRobot(pRobot);
    KinChain.Create(0,0,pRobot->GetLinksCount()-1);
    Encoders.SetSensorsList(pRobot->GetSensors());
    Resize();
  }
}


void SimpleMotionGenerator::GetOutput(Vector & result){
  
  if(result.Size()==mCurrentJointPos.Size())
    result = mCurrentJointPos;
  else{
    cout<<"MOTION GENERATOR: dimension mismatch when getting output, resizing vector!"<<endl;
    result.Resize(mCurrentJointPos.Size());
    result = mCurrentJointPos;
  }





}

