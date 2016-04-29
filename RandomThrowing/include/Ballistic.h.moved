/*
 * Ballistic.h
 *
 *  Created on: Jan 4, 2013
 *      Author: seungsu
 */

#ifndef BALLISTIC_H_
#define BALLISTIC_H_

#include "MathLib.h"

class Ballistic
{
public:
	Ballistic(double dt);
	~Ballistic();

	void SetPosVel(double pos[], double velp[]);
	void PredictNextPos(MathLib::Vector3& pos);

	double CalSmallestError(MathLib::Vector3& TargetPos, MathLib::Vector3& Error);
private:
	double mAirDrag;
	double mDt;

	MathLib::Vector3 mGravity;
	MathLib::Vector3 mPos;
	MathLib::Vector3 mVel;
	MathLib::Vector3 mAccel;
};



#endif /* BALLISTIC_H_ */
