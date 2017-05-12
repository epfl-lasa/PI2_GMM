
#include "DSMotionGenerator.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

DSMotionGenerator::DSMotionGenerator(){
  pRobot=NULL;
  bHaveRobot = false;
  MotionSpeed = 1.0;
  cap = 1.0;
}

void DSMotionGenerator::Resize(){
  int dim = DSmodel.dim;  
  CurrPosV.Resize(dim);
  AttractorPosV.Resize(dim);
  DesVelV.Resize(dim);
  DesPosV.Resize(dim);
  CurrPosRelV.Resize(dim);
  //  cout<<"the DS dimension is "<<dim<<endl;

  if(dim==3)
    nMode = CART_SPACE;
  if(dim==7)
    nMode = JOINT_SPACE;

}

void DSMotionGenerator::SetMaxSpeed(REALTYPE c){
  cap = c;
}


void DSMotionGenerator::Restart(){
  bRestart = true;
}


void DSMotionGenerator::SetMotionSpeed(REALTYPE s){
  MotionSpeed = s;
  bUseExternalSpeed = true;
}

void DSMotionGenerator::SetMotionSpeed(bool b){
  bUseExternalSpeed = b;
}

void DSMotionGenerator::Update(REALTYPE Dt){
  KinChain.Update();
  Encoders.ReadSensors();

  
  switch(nMode){

  case CART_SPACE:
    if(bRestart){
      RunningTime = 0;
      bRestart = false;
      tempV3_1 = pRobot->GetReferenceFrame(pRobot->GetLinksCount()-1,0).GetOrigin();
      tempV3_1.GetVector(CurrPosV);
    }
    
    //    CurrPos = pRobot->GetReferenceFrame(pRobot->GetLinksCount()-1,0).GetOrigin();
    // cout<<"dof count "<<pRobot->GetDOFCount()<<" links count "<<pRobot->GetLinksCount()<<endl;

    CurrPosRelV = CurrPosV;
  
    CurrPosRelV -= AttractorPosV;
    //    CurrPosRel.GetVector(CurrPosRelV);
    
    DSmodel.doRegression(CurrPosRelV,DesVelV);
    if(bUseExternalSpeed){
      DesVelV.Normalize();
      DesVelV*=MotionSpeed;

    }
    else if(DesVelV.Norm()>cap){
      DesVelV.Normalize();
      DesVelV*=cap;
      cout<<"DSMOTIONGENERATOR: capped the speed!"<<endl;
    }
 
    break;

  case JOINT_SPACE:
    if(bRestart){
      RunningTime = 0;
      bRestart = false;
      CurrPosV = Encoders.GetJointAngles();
    }

    //    CurrPosV = Encoders.GetJointAngles();
    CurrPosRelV = CurrPosV;
    CurrPosRelV -= AttractorPosV;
  
    // cout<<"relative pos"<<endl;
    // CurrPosRelV.Print();
    DSmodel.doRegression(CurrPosRelV,DesVelV);

    //Normalize and use the externally provided speed.
    if(bUseExternalSpeed){
      DesVelV.Normalize();
      DesVelV *= MotionSpeed;
    }
    else if(DesVelV.Norm()>cap){
      DesVelV.Normalize();
      DesVelV*=cap;
      cout<<"DSMOTIONGENERATOR: capped the speed!"<<endl;
    }

    break;


  }

  RunningTime+=Dt;

  //integrate 
  DesPosV = CurrPosV; 
  DesVelV.ScaleAddTo(Dt,DesPosV);
  CurrPosV = DesPosV;

}

void DSMotionGenerator::SetAttractorPosition(const Vector & target){
  if(target.Size()==AttractorPosV.Size())
    AttractorPosV = target;
  else{
    cout<<"DSMOTION GENERATOR: dimension mismatch when setting target. Resizing!"<<endl;
    AttractorPosV.Resize(target.Size());
    AttractorPosV = target;
  }
  // for(int i=0;i<3;i++)
  //   AttractorPos(i)=AttractorPosV(i);

  //  AttractorPos.Print();

}

void DSMotionGenerator::LoadModel(const char* txt){
  bool check;
  check = DSmodel.loadParams(txt);
  if(!check)
    cout<<"DSMOTIONGENERATOR: could not find model: "<<txt<<endl;
  Resize();
  //  DSmodel.debug();
}

void DSMotionGenerator::SetRobot(Robot* pRob){
  pRobot=pRob;
  bHaveRobot=true;
  if(pRobot==NULL)
    bHaveRobot=false;
  if(bHaveRobot){
    KinChain.SetRobot(pRobot);
    KinChain.Create(0,0,pRobot->GetLinksCount()-1);
    Encoders.SetSensorsList(pRobot->GetSensors());
    //    Resize();
  }
}

REALTYPE DSMotionGenerator::GetDirectionOfMotion(Vector & result){
  if(result.Size()==DesVelV.Size()){
    result = DesVelV;
    result.Normalize();
  }
  else{
    cout<<"MOTION GENERATOR: dimension mismatch when getting direction of motion, resizing vector!"<<endl;
    result.Resize(DesVelV.Size());
    result = DesVelV;
    result.Normalize();
  }
  return DesVelV.Norm();
}

/*Get the direction of motion and two vectors spanning its orthogonal plane*/
REALTYPE DSMotionGenerator::GetDirectionOfMotion(Vector & MotionDir,Vector & Orth1,Vector & Orth2){
  if(MotionDir.Size()!=DesVelV.Size() || Orth1.Size()!=DesVelV.Size() || Orth2.Size()!=DesVelV.Size()){
    cout<<"MOTION GENERATOR: dimension mismatch when getting direction of motion, resizing vectors!"<<endl;
    MotionDir.Resize(DesVelV.Size());
    Orth1.Resize(DesVelV.Size());
    Orth2.Resize(DesVelV.Size());
  }
  REALTYPE speed;
  speed =  GetDirectionOfMotion(MotionDir);

  REALTYPE th =0.00001;
  if(MotionDir(2)>th){
    Orth1(0)=1;
    Orth1(1)=1;
    Orth1(2)=(-MotionDir(0)-MotionDir(1))/MotionDir(2);
  }
  else if(MotionDir(1)>th){
    Orth1(0)=1;
    Orth1(2)=1;
    Orth1(1)=(-MotionDir(0)-MotionDir(2))/MotionDir(1);
  }
  else if(MotionDir(0)>th){
    Orth1(2)=1;
    Orth1(1)=1;
    Orth1(0)=(-MotionDir(2)-MotionDir(1))/MotionDir(0);
  }
  else{
    //    cout<<"Direction of motion is Zero vector!"<<endl;
  }
  //      Orth1.Zero();
  Orth1.Normalize();
  //      cout<<"Orth1 * MotionDir: "<<MotionDir.Dot(Orth1)<<endl;
  //crossproduct only supported by class vector3..
  tempV3_1.Set(MotionDir);
  tempV3_2.Set(Orth1);
  tempV3_1.Cross(tempV3_2,tempV3_3);
  tempV3_3.GetVector(Orth2);
  
  return speed;
}

void DSMotionGenerator::GetOutput(Vector & result){
  
  if(result.Size()==DesPosV.Size())
    result = DesPosV;
  else{
    cout<<"MOTION GENERATOR: dimension mismatch when getting output, resizing vector!"<<endl;
    result.Resize(DesPosV.Size());
    result = DesPosV;
  }


}

