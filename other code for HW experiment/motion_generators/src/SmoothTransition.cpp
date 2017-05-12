/*
 * SmoothTransition.cpp
 *
 *  Created on: Mar 22, 2012
 *      Author: klas
 */

#include "../include/SmoothTransition.h"

SmoothTransition::SmoothTransition(const Clock * c) {
  // TODO Auto-generated constructor stub
  SmoothTransition();
  SetClock(c);

}

SmoothTransition::SmoothTransition(const Clock* c, int nd){
  mClock = c;
  nDim = nd;
  Resize(nd);
}

SmoothTransition::SmoothTransition(const Clock* c, int nd, double rt){
  mClock = c;
  nDim = nd;
  Resize(nd);
  SetRiseTime(rt);
}


SmoothTransition::~SmoothTransition() {
  // TODO Auto-generated destructor stub
}


void SmoothTransition::SetClock(const Clock* c){
  mClock = c;
}

void SmoothTransition::SetInitial(const Vector & initial){
  startVector = initial;
}
void SmoothTransition::SetTarget(const Vector & target){
  targetVector.Set(target);
}

void SmoothTransition::Reset(){
  StartTime = mClock->GetTime();
}

void SmoothTransition::Resize(int nd){
  nDim = nd;
  startVector.Resize(nDim);
  targetVector.Resize(nDim);
  outputVector.Resize(nDim);
}

void SmoothTransition::Update(){
  CurrTime = mClock->GetTime();
  double RelTime = CurrTime - StartTime;
  if(RelTime<RiseTime){
    double coef1 =  (sin(-PI/2+RelTime/RiseTime*PI)+1)/2;
    outputVector.Zero();
    targetVector.ScaleAddTo(coef1,outputVector);
    startVector.ScaleAddTo((1-coef1),outputVector);
  }
  else{
    outputVector = targetVector;
  }
}

void SmoothTransition::GetOutput(Vector & result){
  result = outputVector;
}

void SmoothTransition::Smooth(const Vector & input, Vector & target){
  if(input.Size()!=target.Size()){
    cout<<"Smoother:: input/target dimension mismatch!"<<endl;
    return;
  }
  else if(input.Size() != nDim){
    cout<<"Smoother is resizing Vectors!"<<endl;
    nDim = input.Size();
    Resize(nDim);
  }
  SetInitial(input);
  SetTarget(target);
  Update();
  target.Set(outputVector);
  return;
}

void SmoothTransition::SetRiseTime(double rt){
  RiseTime = rt;
}
