/*
 * SmoothTransition.h
 *
 *  Created on: Mar 22, 2012
 *      Author: klas
 */

#ifndef SMOOTHTRANSITION_H_
#define SMOOTHTRANSITION_H_


#include "MathLib/MathLib.h"
#include "StdTools/Timer.h"
#include "RobotTools.h"

class SmoothTransition {

const Clock * mClock;
double StartTime;
double CurrTime;
double RiseTime;
unsigned int nDim;

Vector startVector;
Vector targetVector;
Vector outputVector;

public:
	SmoothTransition(){};
	SmoothTransition(const Clock * c);
	SmoothTransition(const Clock * c,int nd);
	SmoothTransition(const Clock * c,int nd,double rt);


	virtual ~SmoothTransition();

	void SetRiseTime(double rt);
	void SetInitial(const Vector & initial);
	void SetTarget(const Vector & target);
	void SetClock(const Clock* c);
	void Resize(int nd);
	void Reset();
	void Update();
	void GetOutput(Vector & res);
	void Smooth(const Vector &, Vector &);
};

#endif /* SMOOTHTRANSITION_H_ */
