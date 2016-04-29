/*
 * FindOptimalJoints.cpp
 *
 *  Created on: Dec 11, 2012
 *      Author: seungsu
 */

#include "Gaussians.h"
#include "sKinematics.h"
#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"

#define KUKA_DOF 7

sKinematics *mSKinematicChain;
IKGroupSolver mIKSolver;
Matrix mJacobian;
Vector mJointVelLimitsUp;
Vector mJointVelLimitsDn;
Vector mJointVelocityLimitsDT;
Vector mJointWeights;


void initKinematics(void)
{
	mSKinematicChain = new sKinematics(KUKA_DOF, 1./500.);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	mSKinematicChain->setDH(0,  0.0,  0.310, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(132.0)*0.99);
	mSKinematicChain->setDH(1,  0.0,  0.000,-M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(132.0)*0.99);
	mSKinematicChain->setDH(2,  0.0,  0.400,-M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(128.0)*0.99);
	mSKinematicChain->setDH(3,  0.0,  0.000, M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(128.0)*0.99);
	mSKinematicChain->setDH(4,  0.0,  0.390, M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(204.0)*0.99);
	mSKinematicChain->setDH(5,  0.0,  0.000,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(184.0)*0.99); // reduced joint angle to save the fingers
	mSKinematicChain->setDH(6,  0.0,  0.217,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(184.0)*0.99);

	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	double T0[4][4];
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++) T0[i][j] = 0.0;

	T0[0][0] = 1;
	T0[1][1] = 1;
	T0[2][2] = 1;
	T0[3][3] = 1;

	mSKinematicChain->setT0(T0);

	// ready for kinematics
	mSKinematicChain->readyForKinematics();

	// variable for ik
	mJacobian.Resize(3,KUKA_DOF);
	mJointWeights.Resize(KUKA_DOF);
}

void initIKSolver(void)
{
	std::vector<unsigned int>  lJonitMapping;
	lJonitMapping.resize(KUKA_DOF);

	// Inverse Kinematics
	mIKSolver.SetSizes(KUKA_DOF);
	mIKSolver.AddSolverItem(3);                 // One solver with 6 constraints
	mIKSolver.SetVerbose(false);                // No comments
	mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
	mIKSolver.Enable(true,0);                   // Enable first solver

	for(int i=0; i<KUKA_DOF; i++)
	{
		lJonitMapping[i] = i;
	}

	mIKSolver.SetDofsIndices(lJonitMapping,0); // Joint maps for first solver


	double maxVel[7];
	mSKinematicChain->getMaxVel(maxVel);

	mJointVelLimitsUp.Resize(KUKA_DOF);
	mJointVelLimitsDn.Resize(KUKA_DOF);
	mJointVelocityLimitsDT.Resize(KUKA_DOF);
	mJointVelocityLimitsDT.Set(maxVel, KUKA_DOF);
	mJointVelocityLimitsDT *= 1./500.;
}

int main(int argc, char** argv)
{

	// for inverse kinematics
	Vector mJointPos(KUKA_DOF);
	Vector ljoints(KUKA_DOF);
	Vector ljointsVel(KUKA_DOF);
	Vector lTargetVel(3);
	Vector llTargetVel(3);
	Vector lMaxThrowingVel(3);
	Matrix lWeight(KUKA_DOF, KUKA_DOF);

	Vector3 lEndPos;

	// for iteration
	int maxdiv = 10;
	int joint, div;
	int i, j[7];

	double jointmin[7];
	double jointmax[7];

	initKinematics();
	initIKSolver();


	FILE *fid;
	fid = fopen("/home/seungsu/data/kuka/reachable_all.txt", "w+");

	for(i=0; i<7; i++){
		jointmin[i] = mSKinematicChain->getMin(i);
		jointmax[i] = mSKinematicChain->getMax(i);
	}

	Vector jmin(7);
	Vector jmax(7);
	jmin.Set(jointmin, 7);
	jmax.Set(jointmax, 7);
	jmin += 0.01;
	jmax -= 0.01;

	mJointPos.Resize(7);

	for( j[0] =0; j[0]<=maxdiv; j[0]++)
	{
		i=0;
		mJointPos(i) = ( jmax(i) - jmin(i) )*j[i]/maxdiv + jmin(i);
		for( j[1] =0; j[1]<=maxdiv; j[1]++)
		{
			i=1;
			mJointPos(i) = ( jmax(i) - jmin(i) )*j[i]/maxdiv + jmin(i);
			for( j[2] =0; j[2]<=maxdiv; j[2]++)
			{
				i=2;
				mJointPos(i) = ( jmax(i) - jmin(i) )*j[i]/maxdiv + jmin(i);

				printf("%d %d %d\n", j[0], j[1], j[2]);
				for( j[3] =0; j[3]<=maxdiv; j[3]++)
				{
					i=3;
					mJointPos(i) = ( jmax(i) - jmin(i) )*j[i]/maxdiv + jmin(i);

					for( j[4] =0; j[4]<=maxdiv; j[4]++)
					{
						i=4;
						mJointPos(i) = ( jmax(i) - jmin(i) )*j[i]/maxdiv + jmin(i);
						for( j[5] =0; j[5]<=maxdiv; j[5]++)
						{
							i=5;
							mJointPos(i) = ( jmax(i) - jmin(i) )*j[i]/maxdiv + jmin(i);
							for( j[6] =0; j[6]<=maxdiv; j[6]++)
							{
								i=6;
								mJointPos(i) = ( jmax(i) - jmin(i) )*j[i]/maxdiv + jmin(i);


								// solve IK;
								mSKinematicChain->setJoints(mJointPos.Array());
								mSKinematicChain->getJacobianPos(mJacobian);
								mSKinematicChain->getEndPos(lEndPos.Array());

								mIKSolver.SetJacobian(mJacobian, 0);

								// Set maximum joint velocity
								for(int i=0;i<KUKA_DOF;i++){
									mJointVelLimitsDn(i) = -mSKinematicChain->getMaxVel(i);
									mJointVelLimitsUp(i) =  mSKinematicChain->getMaxVel(i);

									if( (mJointPos[i]-mSKinematicChain->getMin(i) )< mJointVelocityLimitsDT(i)){
										mJointVelLimitsDn(i) *= (mJointPos[i]-mSKinematicChain->getMin(i))/mJointVelocityLimitsDT(i);
									}else if( (mSKinematicChain->getMax(i)-mJointPos[i])<mJointVelocityLimitsDT(i)){
										mJointVelLimitsUp(i) *= (mSKinematicChain->getMax(i)-mJointPos[i])/mJointVelocityLimitsDT(i);
									}
								}

								mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);


								lTargetVel(0) = -1.0;
								lTargetVel(1) =  0.0;
								lTargetVel(2) =  1.0;

								lTargetVel.Normalize();
								lTargetVel *= 10.0;

								mIKSolver.SetTarget(lTargetVel, 0);
								mIKSolver.Solve();
								ljointsVel = mIKSolver.GetOutput();

								for(int itr=0; itr<2; itr++)
								{
									// residual
									lWeight.Identity();
									for(int i=0;i<KUKA_DOF; i++){
										if( ljointsVel(i) >  mSKinematicChain->getMaxVel(i)-0.1 ){
											lWeight(i,i) = 0.0;
										}
										else if( ljointsVel(i) < -mSKinematicChain->getMaxVel(i)+0.1 ){
											lWeight(i,i) = 0.0;
										}
										else{
											lWeight(i,i) = 1.0;
										}
									}

									llTargetVel = lTargetVel - mJacobian*ljointsVel;

									mIKSolver.SetJacobian(mJacobian*lWeight);
									mIKSolver.SetTarget(llTargetVel);
									mIKSolver.Solve();

									ljointsVel += mIKSolver.GetOutput();

									lMaxThrowingVel = mJacobian *ljointsVel;
								}

								for(int itr=0; itr<KUKA_DOF; itr++ ) fprintf(fid, "%7.5lf ", mJointPos(itr));
								for(int itr=0; itr<3       ; itr++ ) fprintf(fid, "%7.5lf ", lMaxThrowingVel(itr));
								for(int itr=0; itr<KUKA_DOF; itr++ ) fprintf(fid, "%7.5lf ", ljointsVel(itr));
								for(int itr=0; itr<3       ; itr++ ) fprintf(fid, "%7.5lf ", lEndPos(itr));
								fprintf(fid, "\n");
							}
						}
					}
				}
			}
		}
	}

	fclose(fid);
	printf("simulation done \n");






}
