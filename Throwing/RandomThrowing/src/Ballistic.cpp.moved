/*
 * Ballistic.cpp
 *
 *  Created on: Jan 4, 2013
 *      Author: seungsu
 */
#include "Ballistic.h"

Ballistic::Ballistic(double dt)
{
	mDt = dt*0.5;
}

void Ballistic::SetPosVel(double pos[], double vel[])
{
	mPos.Set(pos[0], pos[1], pos[2]);
	mVel.Set(vel[0], vel[1], vel[2]);

	mGravity.Zero();
	mGravity(2) = -9.81;
	mAirDrag =  0.000001;
}

double Ballistic::CalSmallestError(MathLib::Vector3& TargetPos, MathLib::Vector3& Error)
{
	MathLib::Vector3 lError;

	double lDuration = 0.0;

	lError.Set(1000.0);
	while( (lDuration<2.0) && (mPos(0)>TargetPos(0)) )
	{
		mAccel = mGravity - mVel*mAirDrag;
		mPos   += mVel*mDt + mAccel*mDt*mDt*0.5;
		mVel   += mAccel*mDt;
		lDuration += mDt;

		if( (TargetPos-mPos).Norm() > lError.Norm() )
		{
			Error.Set(lError);
			return (lDuration - mDt);
			break;
		}
		else
		{
			lError = TargetPos-mPos;
		}
	}

	Error.Set(lError);
	return (lDuration - mDt);
}

void Ballistic::PredictNextPos(MathLib::Vector3& pos)
{
	// position estimation
	mAccel = mGravity - mVel*mAirDrag;
	mPos   += mVel*mDt + mAccel*mDt*mDt*0.5;
	mVel   += mAccel*mDt;

	pos.Set(mPos);
}

