/*
 * UpperIK.cpp
 *
 *  Created on: Jan 28, 2013
 *      Author: seungsu
 */

#include "sKinematics.h"
#include "MathLib.h"
#include "IKGroupSolver.h"

#define DOF_MAINCHAIN 12
#define DOF_RIGHTCHAIN 9
#define DOF_LEFTCHAIN 9
#define DOF_RIGHTLEGCHAIN 7
#define DOF_LEFTLEGCHAIN 7

#define IK_CON (4*2*3)
#define IK_CON_ARM (3*5)
#define DT (1./500.)

enum ENUM_CHAIN{CHAIN_MAIN=0, CHAIN_LEFT, CHAIN_RIGHT, CHAIN_MAX};
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};

int mNbFrame;
int mNbRbody;

MathLib::Matrix mSkeletonData;
MathLib::Matrix mSkeletonMarkerData;
MathLib::Matrix4 *mSkeletonMatrix;
MathLib::Matrix mSkeletonMarkerMatrix;
MathLib::Matrix mJointTrajectoryMain;
MathLib::Matrix mJointTrajectoryLeft;
MathLib::Matrix mJointTrajectoryRight;
MathLib::Matrix mJointTrajectoryLeftLeg;
MathLib::Matrix mJointTrajectoryRightLeg;


sKinematics *mKinMain;
sKinematics *mKinRightArm;
sKinematics *mKinLeftArm;
sKinematics *mKinRightLeg;
sKinematics *mKinLeftLeg;

sKinematics *mKin4DOF;

MathLib::IKGroupSolver mIKSolverMain;
MathLib::IKGroupSolver mIKSolverLeft;
MathLib::IKGroupSolver mIKSolverRight;
MathLib::IKGroupSolver mIKSolver4DOF;

MathLib::Matrix mJacobian;
MathLib::Vector mJointVelLimitsUp;
MathLib::Vector mJointVelLimitsDn;
MathLib::Vector mJointWeights;

double mLengthMain[5];
double mLengthLeft[4];
double mLengthRight[4];
double mLengthLeftLeg[4];
double mLengthRightLeg[4];

double GetZRotation(MathLib::Vector3& P, ENUM_AXIS axis)
{
	MathLib::Vector3 lP;

	lP = P;
	lP(2) = 0.0;
	lP.Normalize();

	if(axis == AXIS_X)
	{
		if( lP(1) > 0 ) return  acos( MathLib::Vector3::EX.Dot(lP) );
		else            return -acos( MathLib::Vector3::EX.Dot(lP) );
	}
	else if(axis == AXIS_Y)
	{
		if( lP(0) > 0 ) return -acos( MathLib::Vector3::EY.Dot(lP) );
		else            return  acos( MathLib::Vector3::EY.Dot(lP) );
	}
}


void SolveLeft4DOF(MathLib::Vector3& P1, MathLib::Vector3& P2, MathLib::Vector& q)
{
	MathLib::Vector lJoints(3), lOldJoints(3);
	MathLib::Vector lTarget(6);
	MathLib::Vector lCurrent(6);
	MathLib::Vector lTargetVel(6);
	MathLib::Matrix lJacobianPos(3,3);
	MathLib::Matrix lJacobianDir(3,3);
	MathLib::Matrix lJacobian(6,3);
	MathLib::Vector3 lZ;
	MathLib::Matrix4 lT;
	double lOldError, lError;

	P1.Cross(P2, lZ);
	lZ.Normalize();

	lJoints.Zero();
	mKin4DOF->setJoints(lJoints.Array());

	lOldError=100.0;
	for(int itr=0; itr<10; itr++)
	{
		lCurrent.Zero();
		for(int i=0; i<3; i++) lTarget(i  ) =  P1(i);
		for(int i=0; i<3; i++) lTarget(i+3) = -lZ(i);

		mKin4DOF->getEndTMatrix(lT);

		for(int i=0; i<3; i++) lCurrent(i  ) = lT(i,3);
		for(int i=0; i<3; i++) lCurrent(i+3) = lT(i,2);

		lError = (lTarget-lCurrent).Norm();
		//cout << lError <<endl;

		if( (lError < 0.00001) || (lError> lOldError) )
		{
			if(lError> lOldError)
			{
				lJoints.Set(lOldJoints);
				lError = lOldError;
			}
			break;
		}
		else
		{
			lOldError = lError;
		}

		mKin4DOF->getJacobianPos(lJacobianPos);
		mKin4DOF->getJacobianDirection(2, lJacobianDir);

		for(int i=0; i<3; i++) lJacobian.SetRow(lJacobianPos.GetRow(i), i  );
		for(int i=0; i<3; i++) lJacobian.SetRow(lJacobianDir.GetRow(i), i+3);


		mIKSolver4DOF.SetJacobian(lJacobian, 0);
    	lTargetVel = (lTarget - lCurrent)/DT;
    	for(int i=0; i<3; i++) lTargetVel(i+3) *= 0.3;

    	mIKSolver4DOF.SetTarget(lTargetVel, 0);
    	mIKSolver4DOF.Solve();

		lOldJoints.Set(lJoints);
		lJoints += mIKSolver4DOF.GetOutput()*DT;

		mKin4DOF->setJoints(lJoints.Array());
		mKin4DOF->getJoints(lJoints.Array());
	}
	q(0) = lJoints(0);
	q(1) = lJoints(1);
	q(2) = lJoints(2);
	q(3) = acos( P1.Dot(P2)/(P1.Norm()*P2.Norm()) );

}


void LoadData(void)
{
	mSkeletonData.Load("./data/skeleton_catching.txt");
	mSkeletonMarkerData.Load("./data/marker_catching.txt");
	//mSkeletonData.Load("./data/skeleton_00.txt");
	//mSkeletonMarkerData.Load("./data/marker_00.txt");

	mNbFrame = mSkeletonData.RowSize();
	mNbRbody = mSkeletonData.ColumnSize()/9;

	mSkeletonMatrix = (MathLib::Matrix4 *)malloc(mNbFrame*sizeof(MathLib::Matrix4)*24);
	mJointTrajectoryMain.Resize(mNbFrame, DOF_MAINCHAIN);
	mJointTrajectoryLeft.Resize(mNbFrame, DOF_LEFTCHAIN);
	mJointTrajectoryRight.Resize(mNbFrame, DOF_RIGHTCHAIN);
	mJointTrajectoryLeftLeg.Resize(mNbFrame, DOF_LEFTLEGCHAIN);
	mJointTrajectoryRightLeg.Resize(mNbFrame, DOF_RIGHTLEGCHAIN);

	mSkeletonMarkerMatrix.Resize(mNbFrame, 9*24);
}

void ConvertCoordinate(void)
{
	MathLib::Matrix4 lBWM, lBaseOri;
	MathLib::Matrix3 lOri;
	MathLib::Vector3 lX, lY, lZ;
	MathLib::Matrix4 lWSkelM;
	MathLib::Vector3  lV1;
	double lAngle;

	lBWM.Identity();
	lBaseOri.Identity();

	/*
	FILE *fid       = fopen("./data/skeleton2.txt", "w+");
	FILE *fidmarker = fopen("./data/skeleton2_marker.txt", "w+");
	FILE *fidbase   = fopen("./data/skeleton2_base.txt", "w+");
	FILE *fidball   = fopen("./data/skeleton2_ball.txt", "w+");
	*/
	FILE *fid       = fopen("./data/catching.txt", "w+");
	FILE *fidmarker = fopen("./data/catching_marker.txt", "w+");
	FILE *fidbase   = fopen("./data/catching_base.txt", "w+");
	FILE *fidball   = fopen("./data/catching_ball.txt", "w+");

	for(int frame=0; frame<mNbFrame; frame++)
	{
		for(int i=0; i<mNbRbody; i++)
		{
			lWSkelM.Identity();
			lWSkelM(0,3) = mSkeletonData(frame,0+i*9);
			lWSkelM(1,3) = mSkeletonData(frame,1+i*9);
			lWSkelM(2,3) = mSkeletonData(frame,2+i*9);

			lY(0) = mSkeletonData(frame,3+i*9);
			lY(1) = mSkeletonData(frame,4+i*9);
			lY(2) = mSkeletonData(frame,5+i*9);

			lZ(0) = mSkeletonData(frame,6+i*9);
			lZ(1) = mSkeletonData(frame,7+i*9);
			lZ(2) = mSkeletonData(frame,8+i*9);

			lX = lY.Cross(lZ);

			for(int row=0; row<3; row++)
			{
				lWSkelM(row,0) = lX(row);
				lWSkelM(row,1) = lY(row);
				lWSkelM(row,2) = lZ(row);
			}

			if( i== 0)
			{
				for(int i=0; i<3; i++) lY(i) = mSkeletonData(frame,i+9*16) - mSkeletonData(frame,i);
				for(int i=0; i<3; i++) lZ(i) = mSkeletonData(frame,i+9*1 ) - mSkeletonData(frame,i);

				lY.Normalize();
				lZ.Normalize();
				lX = lY.Cross(lZ);
				lX.Normalize();
				lZ = lX.Cross(lY);
				lZ.Normalize();

				for(int row=0; row<3; row++)
				{
					lBaseOri(row,0) = lX(row);
					lBaseOri(row,1) = lY(row);
					lBaseOri(row,2) = lZ(row);

					lBaseOri(row,3) = mSkeletonData(frame,row);
				}

				lBaseOri.InverseTransformation(lBWM);
				lBaseOri = lBWM*lWSkelM;
				lBaseOri = lBaseOri.InverseTransformation();
			}

			lWSkelM = lBWM*lWSkelM*lBaseOri;

			mSkeletonMatrix[frame*24+i].Set(lWSkelM);
			lBWM.GetOrientation(lOri);
			for(int row=0; row<3; row++) lV1(row) = mSkeletonMarkerData(frame, row  +9*i);
			lV1 = lOri*lV1;
			for(int row=0; row<3; row++) mSkeletonMarkerMatrix(frame, i*9+row) =lV1(row)+lBWM(row,3);

			for(int row=0; row<3; row++) lV1(row) = mSkeletonMarkerData(frame, row+3+9*i);
			lV1 = lOri*lV1;
			for(int row=0; row<3; row++) mSkeletonMarkerMatrix(frame, i*9+row+3) =lV1(row)+lBWM(row,3);

			for(int row=0; row<3; row++) lV1(row) = mSkeletonMarkerData(frame, row+6+9*i);
			lV1 = lOri*lV1;
			for(int row=0; row<3; row++) mSkeletonMarkerMatrix(frame, i*9+row+6) =lV1(row)+lBWM(row,3);



			fprintf(fid, "%lf %lf %lf %lf %lf %lf %lf %lf %lf ",
					lWSkelM(0,3), lWSkelM(1,3), lWSkelM(2,3),
					lWSkelM(0,0), lWSkelM(1,0), lWSkelM(2,0),
					lWSkelM(0,1), lWSkelM(1,1), lWSkelM(2,1) );

			for(int row=0; row<9; row++) fprintf(fidmarker, "%lf ", mSkeletonMarkerMatrix(frame, i*9+row));
		}
		fprintf(fid, "\n");

		//for(int row=0; row<3; row++) lV1(row) = mSkeletonMarkerData(frame, row+ 9*24);
		//lV1 = lOri*lV1;
		//for(int row=0; row<3; row++) fprintf(fidmarker, "%lf ", lV1(row)+lBWM(row,3));
		fprintf(fidmarker, "%lf \n", mSkeletonMarkerData(frame, 24*9+3));

		for(int row=0; row<3; row++) fprintf(fidbase, "%lf ", lBWM(row,0) );
		for(int row=0; row<3; row++) fprintf(fidbase, "%lf ", lBWM(row,1) );
		for(int row=0; row<3; row++) fprintf(fidbase, "%lf ", lBWM(row,2) );
		for(int row=0; row<3; row++) fprintf(fidbase, "%lf ", lBWM(row,3) );
		fprintf(fidbase, "\n ");

		for(int row=0; row<3; row++) fprintf(fidball, "%lf ", mSkeletonMarkerData(frame, row+ 9*24) );
		fprintf(fidball, "\n ");
	}
	fclose(fid);
	fclose(fidmarker);
	fclose(fidbase);
	fclose(fidball);

}


void initKinematicsLeftLeg(void)
{
	mLengthLeftLeg[0] = (mSkeletonMatrix[17-1].GetTranslation()-mSkeletonMatrix[   0].GetTranslation()).Norm() ;
	mLengthLeftLeg[1] = (mSkeletonMatrix[18-1].GetTranslation()-mSkeletonMatrix[17-1].GetTranslation()).Norm() ;
	mLengthLeftLeg[2] = (mSkeletonMatrix[19-1].GetTranslation()-mSkeletonMatrix[18-1].GetTranslation()).Norm() ;
	mLengthLeftLeg[3] = (mSkeletonMatrix[20-1].GetTranslation()-mSkeletonMatrix[19-1].GetTranslation()).Norm() ;

	mKinLeftLeg = new sKinematics(DOF_LEFTCHAIN+1, 1./500.);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	mKinLeftLeg->setDH( 0,              0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftLeg->setDH( 1,              0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftLeg->setDH( 2,mLengthLeftLeg[1],            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftLeg->setDH( 3,mLengthLeftLeg[2],            0.0,    0.0,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftLeg->setDH( 4,              0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftLeg->setDH( 5,              0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftLeg->setDH( 6,              0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftLeg->setDH( 7,mLengthLeftLeg[3],            0.0,    0.0, M_PI_2, 0, -M_PI, M_PI, M_PI*2*10);

	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	MathLib::Matrix4 T0;
	T0.Zero();
	T0(2,0) = -1.0;
	T0(0,1) =  1.0;
	T0(1,2) = -1.0;
	T0(1,3) =  mLengthLeftLeg[0];
	T0(3,3) =  1.0;

	mKinLeftLeg->setT0(T0);

	// ready for kinematics
	mKinLeftLeg->readyForKinematics();
}

void initKinematicsRightLeg(void)
{
	mLengthRightLeg[0] = (mSkeletonMatrix[21-1].GetTranslation()-mSkeletonMatrix[   0].GetTranslation()).Norm() ;
	mLengthRightLeg[1] = (mSkeletonMatrix[22-1].GetTranslation()-mSkeletonMatrix[21-1].GetTranslation()).Norm() ;
	mLengthRightLeg[2] = (mSkeletonMatrix[23-1].GetTranslation()-mSkeletonMatrix[22-1].GetTranslation()).Norm() ;
	mLengthRightLeg[3] = (mSkeletonMatrix[24-1].GetTranslation()-mSkeletonMatrix[23-1].GetTranslation()).Norm() ;

	mKinRightLeg = new sKinematics(DOF_RIGHTLEGCHAIN+1, 1./500.);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	mKinRightLeg->setDH( 0,               0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightLeg->setDH( 1,               0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightLeg->setDH( 2,mLengthRightLeg[1],            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightLeg->setDH( 3,mLengthRightLeg[2],            0.0,    0.0,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightLeg->setDH( 4,               0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightLeg->setDH( 5,               0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightLeg->setDH( 6,               0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightLeg->setDH( 7,mLengthRightLeg[3],            0.0,    0.0, M_PI_2, 0, -M_PI, M_PI, M_PI*2*10);

	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	MathLib::Matrix4 T0;
	T0.Zero();
	T0(2,0) = -1.0;
	T0(0,1) =  1.0;
	T0(1,2) = -1.0;
	T0(1,3) = -mLengthRightLeg[0];
	T0(3,3) =  1.0;

	mKinRightLeg->setT0(T0);

	// ready for kinematics
	mKinRightLeg->readyForKinematics();
}

void initKinematicsLeft(void)
{
	mLengthLeft[0] = (mSkeletonMatrix[ 8-1].GetTranslation()-mSkeletonMatrix[ 7-1].GetTranslation()).Norm() ;
	mLengthLeft[1] = (mSkeletonMatrix[ 9-1].GetTranslation()-mSkeletonMatrix[ 8-1].GetTranslation()).Norm() ;
	mLengthLeft[2] = (mSkeletonMatrix[10-1].GetTranslation()-mSkeletonMatrix[ 9-1].GetTranslation()).Norm() ;
	mLengthLeft[3] = (mSkeletonMatrix[11-1].GetTranslation()-mSkeletonMatrix[10-1].GetTranslation()).Norm() ;

	mKinLeftArm = new sKinematics(DOF_LEFTCHAIN+1, 1./500.);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	mKinLeftArm->setDH( 0,            0.0,            0.0,-M_PI_2,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 1, mLengthLeft[0],            0.0,    0.0,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 2,            0.0,            0.0,-M_PI_2,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 3,            0.0,            0.0,-M_PI_2,-M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 4,            0.0, mLengthLeft[1], M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 5, mLengthLeft[2],            0.0,    0.0, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 6,            0.0,            0.0, M_PI_2,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 7,            0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 8,            0.0, mLengthLeft[3],    0.0,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinLeftArm->setDH( 9,            0.0,            0.0, M_PI_2, M_PI_2, 0, -M_PI, M_PI, M_PI*2*10);

	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	MathLib::Matrix4 T0;
	T0.Identity();

	mKinLeftArm->setT0(T0);

	// ready for kinematics
	mKinLeftArm->readyForKinematics();

	// variable for ik
	mJacobian.Resize(IK_CON_ARM, DOF_LEFTCHAIN);
	mJointWeights.Resize(DOF_LEFTCHAIN);
}

void initKinematicsRight(void)
{
	mLengthRight[0] = (mSkeletonMatrix[13-1].GetTranslation()-mSkeletonMatrix[12-1].GetTranslation()).Norm() ;
	mLengthRight[1] = (mSkeletonMatrix[14-1].GetTranslation()-mSkeletonMatrix[13-1].GetTranslation()).Norm() ;
	mLengthRight[2] = (mSkeletonMatrix[15-1].GetTranslation()-mSkeletonMatrix[14-1].GetTranslation()).Norm() ;
	mLengthRight[3] = (mSkeletonMatrix[16-1].GetTranslation()-mSkeletonMatrix[15-1].GetTranslation()).Norm() ;

	mKinRightArm = new sKinematics(DOF_RIGHTCHAIN+1, 1./500.);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	mKinRightArm->setDH( 0,             0.0,             0.0,-M_PI_2,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 1,-mLengthRight[0],             0.0,    0.0,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 2,             0.0,             0.0,-M_PI_2,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 3,             0.0,             0.0,-M_PI_2,-M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 4,             0.0,-mLengthRight[1], M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 5,-mLengthRight[2],             0.0,    0.0, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 6,             0.0,             0.0, M_PI_2,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 7,             0.0,             0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 8,             0.0,-mLengthRight[3],    0.0,    0.0, 1, -M_PI, M_PI, M_PI*2*10);
	mKinRightArm->setDH( 9,             0.0,             0.0, M_PI_2, M_PI_2, 0, -M_PI, M_PI, M_PI*2*10);

	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	MathLib::Matrix4 T0;
	T0.Identity();

	mKinRightArm->setT0(T0);

	// ready for kinematics
	mKinRightArm->readyForKinematics();

	// variable for ik
	mJacobian.Resize(IK_CON_ARM, DOF_RIGHTCHAIN);
	mJointWeights.Resize(DOF_RIGHTCHAIN);
}

void initKinematicsMain(void)
{
	mLengthMain[0] = (mSkeletonMatrix[1].GetTranslation()-mSkeletonMatrix[0].GetTranslation()).Norm() ;
	mLengthMain[1] = (mSkeletonMatrix[2].GetTranslation()-mSkeletonMatrix[1].GetTranslation()).Norm() ;
	mLengthMain[2] = (mSkeletonMatrix[3].GetTranslation()-mSkeletonMatrix[2].GetTranslation()).Norm() ;
	mLengthMain[3] = (mSkeletonMatrix[4].GetTranslation()-mSkeletonMatrix[3].GetTranslation()).Norm() ;
	mLengthMain[4] = (mSkeletonMatrix[5].GetTranslation()-mSkeletonMatrix[4].GetTranslation()).Norm() ;

	mKinMain = new sKinematics(DOF_MAINCHAIN+1, 1./500.);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	mKinMain->setDH( 0, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinMain->setDH( 1, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinMain->setDH( 2, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);

	mKinMain->setDH( 3, 0.0, mLengthMain[1], M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinMain->setDH( 4, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinMain->setDH( 5, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);

	mKinMain->setDH( 6, 0.0, mLengthMain[2], M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinMain->setDH( 7, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinMain->setDH( 8, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);

	mKinMain->setDH( 9, 0.0, mLengthMain[3], M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinMain->setDH(10, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
	mKinMain->setDH(11, 0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);

	mKinMain->setDH(12, 0.0, mLengthMain[4],    0.0,    0.0, 0, -M_PI, M_PI, M_PI*2*10);

	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	double T0[4][4];
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++) T0[i][j] = 0.0;

	T0[0][0] = 1;
	T0[1][1] = 1;
	T0[2][2] = 1;
	T0[3][3] = 1;

	T0[2][3] = mLengthMain[0];
	mKinMain->setT0(T0);

	// ready for kinematics
	mKinMain->readyForKinematics();

	// variable for ik
	mJacobian.Resize(IK_CON, DOF_MAINCHAIN);
	mJointWeights.Resize(DOF_MAINCHAIN);
}


void initIKSolver4DOF(char lorr)
{
	MathLib::IndicesVector mJointMapping;

	mIKSolver4DOF.SetSizes(3);
	mIKSolver4DOF.Resize();
	mIKSolver4DOF.AddSolverItem(6);    // One solver with 6 constraints
	mIKSolver4DOF.SetVerbose(false);                // No comments
	mIKSolver4DOF.SetThresholds(0.0001,0.00001);    // Singularities thresholds
	mIKSolver4DOF.Enable(true,0);                   // Enable first solver

	mJointMapping.clear();
	for(int i=0; i<3; i++){
		mJointMapping.push_back(i);
	}

	mIKSolver4DOF.SetDofsIndices(mJointMapping,0); // Joint maps for first solver

	mJointVelLimitsUp.Resize(3);
	mJointVelLimitsDn.Resize(3);

	mKin4DOF = new sKinematics(3, 1./500.);

	if( lorr == 'l')
	{
		mKin4DOF->setDH(0,            0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
		mKin4DOF->setDH(1,            0.0,            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);
		mKin4DOF->setDH(2, mLengthLeft[1],            0.0, M_PI_2, M_PI_2, 1, -M_PI, M_PI, M_PI*2*10);

		mKin4DOF->getMaxVel(mJointVelLimitsDn.Array());
		mKin4DOF->getMaxVel(mJointVelLimitsUp.Array());
	}

	mJointVelLimitsDn *= -1.0;

	mKin4DOF->readyForKinematics();
}

void initIKSolver(ENUM_CHAIN chain)
{
	int lDofChain;
	int lNbConstraints;
	MathLib::IndicesVector mJointMapping;


	if( chain == CHAIN_MAIN )
	{
		lDofChain = DOF_MAINCHAIN;
		lNbConstraints = IK_CON;

		// Inverse Kinematics
		mIKSolverMain.SetSizes(lDofChain);
		mIKSolverMain.Resize();
		mIKSolverMain.AddSolverItem(lNbConstraints);    // One solver with 6 constraints
		mIKSolverMain.SetVerbose(false);                // No comments
		mIKSolverMain.SetThresholds(0.0001,0.00001);    // Singularities thresholds
		mIKSolverMain.Enable(true,0);                   // Enable first solver

		mJointMapping.clear();
		for(int i=0; i<lDofChain; i++){
			mJointMapping.push_back(i);
		}

		mIKSolverMain.SetDofsIndices(mJointMapping,0); // Joint maps for first solver

	}
	else if( chain == CHAIN_LEFT )
	{
		lDofChain = DOF_LEFTCHAIN;
		lNbConstraints = IK_CON_ARM;

		// Inverse Kinematics
		mIKSolverLeft.SetSizes(lDofChain);
		mIKSolverLeft.Resize();
		mIKSolverLeft.AddSolverItem(lNbConstraints);    // One solver with 6 constraints
		mIKSolverLeft.SetVerbose(false);                // No comments
		mIKSolverLeft.SetThresholds(0.0001,0.00001);    // Singularities thresholds
		mIKSolverLeft.Enable(true,0);                   // Enable first solver

		mJointMapping.clear();
		for(int i=0; i<lDofChain; i++){
			mJointMapping.push_back(i);
		}

		mIKSolverLeft.SetDofsIndices(mJointMapping,0); // Joint maps for first solver
	}
	else if( chain == CHAIN_RIGHT )
	{
		lDofChain = DOF_RIGHTCHAIN;
		lNbConstraints = IK_CON_ARM;

		// Inverse Kinematics
		mIKSolverRight.SetSizes(lDofChain);
		mIKSolverRight.Resize();
		mIKSolverRight.AddSolverItem(lNbConstraints);    // One solver with 6 constraints
		mIKSolverRight.SetVerbose(false);                // No comments
		mIKSolverRight.SetThresholds(0.0001,0.00001);    // Singularities thresholds
		mIKSolverRight.Enable(true,0);                   // Enable first solver

		mJointMapping.clear();
		for(int i=0; i<lDofChain; i++){
			mJointMapping.push_back(i);
		}

		mIKSolverRight.SetDofsIndices(mJointMapping,0); // Joint maps for first solver
	}

	mJointVelLimitsUp.Resize(lDofChain);
	mJointVelLimitsDn.Resize(lDofChain);

	if( chain == CHAIN_MAIN )
	{
		mKinMain->getMaxVel(mJointVelLimitsDn.Array());
		mKinMain->getMaxVel(mJointVelLimitsUp.Array());
	}
	else if( chain == CHAIN_LEFT )
	{
		mKinLeftArm->getMaxVel(mJointVelLimitsDn.Array());
		mKinLeftArm->getMaxVel(mJointVelLimitsUp.Array());
	}
	else if( chain == CHAIN_RIGHT )
	{
		mKinRightArm->getMaxVel(mJointVelLimitsDn.Array());
		mKinRightArm->getMaxVel(mJointVelLimitsUp.Array());
	}

	mJointVelLimitsDn *= -1.0;
}


void SolveIKLeftArmManual(void)
{
	MathLib::Vector lJoints(DOF_LEFTCHAIN);
	MathLib::Vector lOldJoints(DOF_LEFTCHAIN);
	MathLib::Vector lTarget(IK_CON_ARM);
	MathLib::Vector lTargetVel(IK_CON_ARM);
	MathLib::Vector lCurrent(IK_CON_ARM);
	MathLib::Vector lq(4);

	MathLib::Vector lJointsMain(DOF_MAINCHAIN);
	MathLib::Vector lPos(3);
	MathLib::Matrix4 lT0, lT1;
	MathLib::Vector3 lV1, lV2, lV3;

	int lDirAxis = 1;

	double lError;

	lJoints.Zero();
	mKinLeftArm->setJoints(lJoints.Array());

	int lIndex[4];
	lIndex[0] = 1;
	lIndex[1] = 4;
	lIndex[2] = 5;
	lIndex[3] = 9;

	mIKSolver4DOF.SetLimits(mJointVelLimitsDn, mJointVelLimitsUp);

	for(int frame=0; frame<mNbFrame; frame++)
	//for(int frame=0; frame<1; frame++)
	{
		lJoints.Zero();

		lJointsMain.Set(mJointTrajectoryMain.GetRow(frame));
		lJointsMain(6) = 0.0;
		mKinMain->setJoints(lJointsMain.Array());
		mKinMain->getLinkTMatrix(6, lT0);

		mKinLeftArm->setT0(lT0);
		mKinLeftArm->readyForKinematics();

		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[7+24*frame](i,3);

		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lJoints(0) = GetZRotation(lV1, AXIS_X);
		mKinLeftArm->setJoints(lJoints.Array());

		mKinLeftArm->getLinkTMatrix(0, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[7+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lJoints(1) = GetZRotation(lV1, AXIS_X);
		mKinLeftArm->setJoints(lJoints.Array());

		mKinLeftArm->getLinkTMatrix(1, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[8+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lJoints(2) = GetZRotation(lV1, AXIS_X);
		mKinLeftArm->setJoints(lJoints.Array());

		mKinLeftArm->getLinkTMatrix(2, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[8+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lJoints(3) = GetZRotation(lV1, AXIS_X);
		mKinLeftArm->setJoints(lJoints.Array());

		mKinLeftArm->getLinkTMatrix(3, lT0);
		mKinLeftArm->getEndPos(3, lV3.Array());
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[8+24*frame](i,3) - lV3(i);
		for(int i=0; i<3; i++) lV2(i) = mSkeletonMatrix[9+24*frame](i,3) - mSkeletonMatrix[8+24*frame](i,3);
		lV1.Normalize();
		lV2.Normalize();
		lV1.Cross(lV2, lV3);
		lV3.Normalize();
		lV3 = -lV3;

		lT0.InverseTransformation(lT1);
		lV3 = lT1.GetOrientation()*lV3;
		lJoints(4) = GetZRotation(lV3, AXIS_X);
		mKinLeftArm->setJoints(lJoints.Array());


		mKinLeftArm->getEndPos(1, lV1.Array());
		mKinLeftArm->getEndPos(4, lV2.Array());
		for(int i=0; i<3; i++) lV3(i) = mSkeletonMatrix[9+24*frame](i,3);

		lV1 = (lV2-lV1);
		lV2 = (lV3-lV2);

		lV1.Normalize();
		lV2.Normalize();
		lJoints(5) = -acos(lV1.Dot(lV2));
		mKinLeftArm->setJoints(lJoints.Array());

		// last 3 joints

		mKinLeftArm->getLinkTMatrix(5, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[10+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = (lT1.GetOrientation()*lV1) + lT1.GetTranslation();
		lJoints(6) = GetZRotation(lV1, AXIS_X);
		mKinLeftArm->setJoints(lJoints.Array());


		mKinLeftArm->getLinkTMatrix(6, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[10+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = (lT1.GetOrientation()*lV1) + lT1.GetTranslation();
		lJoints(7) = GetZRotation(lV1, AXIS_X);
		mKinLeftArm->setJoints(lJoints.Array());

		mKinLeftArm->getLinkTMatrix(8, lT0);

		for(int i=0; i<3; i++) lV1(i) = mSkeletonMarkerMatrix(frame, 9*9+i  ) - mSkeletonMarkerMatrix(frame, 9*9+i+3);
		for(int i=0; i<3; i++) lV2(i) = mSkeletonMarkerMatrix(frame, 9*9+i+3) - mSkeletonMarkerMatrix(frame, 9*9+i+6);
		//printf("%lf %lf %lf , %lf %lf %lf \n", lV1(0), lV1(1), lV1(2), lV2(0), lV2(1), lV2(2) );

		lV1.Normalize();
		lV2.Normalize();
		lV2.Cross(lV1, lV3);

		lT0.InverseTransformation(lT1);
		lV3 = (lT1.GetOrientation()*lV3);
		lJoints(8) = GetZRotation(lV3, AXIS_X);
		mKinLeftArm->setJoints(lJoints.Array());

		mJointTrajectoryLeft.SetRow( lJoints, frame);
		//cout << frame << " " << mNbFrame << " " << lError << endl;
	}
}


void SolveIKRightArmManual(void)
{
	MathLib::Vector lJoints(DOF_RIGHTCHAIN);
	MathLib::Vector lOldJoints(DOF_RIGHTCHAIN);
	MathLib::Vector lTarget(IK_CON_ARM);
	MathLib::Vector lTargetVel(IK_CON_ARM);
	MathLib::Vector lCurrent(IK_CON_ARM);
	MathLib::Vector lq(4);

	MathLib::Vector lJointsMain(DOF_MAINCHAIN);
	MathLib::Vector lPos(3);
	MathLib::Matrix4 lT0, lT1;
	MathLib::Vector3 lV1, lV2, lV3;

	int lDirAxis = 1;

	double lError;

	lJoints.Zero();
	mKinRightArm->setJoints(lJoints.Array());

	for(int frame=0; frame<mNbFrame; frame++)
	{
		lJoints.Zero();

		lJointsMain.Set(mJointTrajectoryMain.GetRow(frame));
		lJointsMain(6) = 0.0;
		mKinMain->setJoints(lJointsMain.Array());
		mKinMain->getLinkTMatrix(6, lT0);

		mKinRightArm->setT0(lT0);
		mKinRightArm->readyForKinematics();

		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[7+5+24*frame](i,3);

		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lV1 = -lV1;
		lJoints(0) = GetZRotation(lV1, AXIS_X);
		mKinRightArm->setJoints(lJoints.Array());

		mKinRightArm->getLinkTMatrix(0, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[7+5+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lV1 = -lV1;
		lJoints(1) = GetZRotation(lV1, AXIS_X);
		mKinRightArm->setJoints(lJoints.Array());

		mKinRightArm->getLinkTMatrix(1, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[8+5+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lV1 = -lV1;
		lJoints(2) = GetZRotation(lV1, AXIS_X);
		mKinRightArm->setJoints(lJoints.Array());

		mKinRightArm->getLinkTMatrix(2, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[8+5+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lV1 = -lV1;
		lJoints(3) = GetZRotation(lV1, AXIS_X);
		mKinRightArm->setJoints(lJoints.Array());

		mKinRightArm->getLinkTMatrix(3, lT0);
		mKinRightArm->getEndPos(3, lV3.Array());
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[8+5+24*frame](i,3) - lV3(i);
		for(int i=0; i<3; i++) lV2(i) = mSkeletonMatrix[9+5+24*frame](i,3) - mSkeletonMatrix[8+5+24*frame](i,3);
		lV1.Normalize();
		lV2.Normalize();
		lV1.Cross(lV2, lV3);
		lV3.Normalize();

		lT0.InverseTransformation(lT1);
		lV3 = lT1.GetOrientation()*lV3;
		lJoints(4) = GetZRotation(lV3, AXIS_X);
		mKinRightArm->setJoints(lJoints.Array());


		mKinRightArm->getEndPos(1, lV1.Array());
		mKinRightArm->getEndPos(4, lV2.Array());
		for(int i=0; i<3; i++) lV3(i) = mSkeletonMatrix[9+5+24*frame](i,3);
		if( frame == 0 ){
			lV1.Print();
			lV2.Print();
			lV3.Print();
		}


		lV1 = (lV2-lV1);
		lV2 = (lV3-lV2);

		lV1.Normalize();
		lV2.Normalize();
		lJoints(5) = acos(lV1.Dot(lV2));
		mKinRightArm->setJoints(lJoints.Array());

		// last 3 joints

		mKinRightArm->getLinkTMatrix(5, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[10+5+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = (lT1.GetOrientation()*lV1) + lT1.GetTranslation();
		lV1 = -lV1;
		lJoints(6) = GetZRotation(lV1, AXIS_X);
		mKinRightArm->setJoints(lJoints.Array());

		mKinRightArm->getLinkTMatrix(6, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[10+5+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = (lT1.GetOrientation()*lV1) + lT1.GetTranslation();
		lV1 = -lV1;
		lJoints(7) = GetZRotation(lV1, AXIS_X);
		mKinRightArm->setJoints(lJoints.Array());

		mKinRightArm->getLinkTMatrix(8, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMarkerMatrix(frame, 14*9+i  ) - mSkeletonMarkerMatrix(frame, 14*9+i+3);
		for(int i=0; i<3; i++) lV2(i) = mSkeletonMarkerMatrix(frame, 14*9+i+3) - mSkeletonMarkerMatrix(frame, 14*9+i+6);
		//printf("%lf %lf %lf , %lf %lf %lf \n", lV1(0), lV1(1), lV1(2), lV2(0), lV2(1), lV2(2) );

		lV1.Normalize();
		lV2.Normalize();
		lV2.Cross(lV1, lV3);
		lV3 = - lV3;

		lT0.InverseTransformation(lT1);
		lV3 = (lT1.GetOrientation()*lV3);
		lJoints(8) = GetZRotation(lV3, AXIS_X);


		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[10+5+24*frame](i,0);
		lT0.InverseTransformation(lT1);
		lV1 = (lT1.GetOrientation()*lV1);
		lJoints(8) = GetZRotation(lV1, AXIS_X);
		mKinRightArm->setJoints(lJoints.Array());

		mJointTrajectoryRight.SetRow( lJoints, frame);
		//cout << frame << " " << mNbFrame << " " << lError << endl;
	}
}



void SolveIKLeftLegManual(void)
{
	MathLib::Vector lJoints(DOF_LEFTLEGCHAIN);
	MathLib::Vector lPos(3);
	MathLib::Matrix4 lT0, lT1;
	MathLib::Vector3 lV1, lV2, lV3;

	int lDirAxis = 1;
	double lError;

	lJoints.Zero();
	mKinLeftLeg->setJoints(lJoints.Array());


	for(int frame=0; frame<mNbFrame; frame++)
	{
		lJoints.Zero();
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[18-1+24*frame](i,3);

		mKinLeftLeg->getT0(lT0);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lJoints(0) = GetZRotation(lV1, AXIS_X);
		mKinLeftLeg->setJoints(lJoints.Array());

		mKinLeftLeg->getLinkTMatrix(0, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[20-1+24*frame](i,3) - mSkeletonMatrix[19-1+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1;
		lV1.Normalize();
		lJoints(1) = GetZRotation(lV1, AXIS_X);
		mKinLeftLeg->setJoints(lJoints.Array());

		mKinLeftLeg->getLinkTMatrix(1, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[18-1+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lJoints(2) = GetZRotation(lV1, AXIS_Y);
		mKinLeftLeg->setJoints(lJoints.Array());

		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[18-1+24*frame](i,3) - mSkeletonMatrix[17-1+24*frame](i,3);
		for(int i=0; i<3; i++) lV2(i) = mSkeletonMatrix[19-1+24*frame](i,3) - mSkeletonMatrix[18-1+24*frame](i,3);
		lV1.Normalize();
		lV2.Normalize();
		lJoints(3) = -acos(lV1.Dot(lV2));
		mKinLeftLeg->setJoints(lJoints.Array());

		mKinLeftLeg->getLinkTMatrix(3, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[20-1+24*frame](i,3) - lT0(i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1;
		lJoints(4) = GetZRotation(lV1, AXIS_Y);
		mKinLeftLeg->setJoints(lJoints.Array());

		mKinLeftLeg->getLinkTMatrix(4, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[20-1+24*frame](i,3) - lT0(i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1;
		lJoints(5) = GetZRotation(lV1, AXIS_X);
		mKinLeftLeg->setJoints(lJoints.Array());

		mKinLeftLeg->getLinkTMatrix(6, lT0);
		for(int i=0; i<3; i++) lV1(i) = -mSkeletonMatrix[20-1+24*frame](i,1);
		for(int i=0; i<3; i++) lV2(i) =  lT0(i,2);
		lV1.Normalize();
		lV2.Normalize();
		lJoints(6) = acos(lV1.Dot(lV2));

		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[20-1+24*frame](i,2);
		lV1.Normalize();
		if(lV1.Dot(lV2) > 0) lJoints(6) = -lJoints(6);

		mJointTrajectoryLeftLeg.SetRow( lJoints, frame);
		//cout << frame << " " << mNbFrame << " " << lError << endl;
	}
}


void SolveIKRightLegManual(void)
{
	MathLib::Vector lJoints(DOF_RIGHTLEGCHAIN);
	MathLib::Vector lPos(3);
	MathLib::Matrix4 lT0, lT1;
	MathLib::Vector3 lV1, lV2, lV3;

	int lDirAxis = 1;
	double lError;

	lJoints.Zero();
	mKinRightLeg->setJoints(lJoints.Array());


	for(int frame=0; frame<mNbFrame; frame++)
	{
		lJoints.Zero();
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[18+4-1+24*frame](i,3);

		mKinRightLeg->getT0(lT0);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lJoints(0) = GetZRotation(lV1, AXIS_X);
		mKinRightLeg->setJoints(lJoints.Array());

		mKinRightLeg->getLinkTMatrix(0, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[20+4-1+24*frame](i,3) - mSkeletonMatrix[19+4-1+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1;
		lV1.Normalize();
		lJoints(1) = GetZRotation(lV1, AXIS_X);
		mKinRightLeg->setJoints(lJoints.Array());

		mKinRightLeg->getLinkTMatrix(1, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[18+4-1+24*frame](i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
		lJoints(2) = GetZRotation(lV1, AXIS_Y);
		mKinRightLeg->setJoints(lJoints.Array());

		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[18+4-1+24*frame](i,3) - mSkeletonMatrix[17+4-1+24*frame](i,3);
		for(int i=0; i<3; i++) lV2(i) = mSkeletonMatrix[19+4-1+24*frame](i,3) - mSkeletonMatrix[18+4-1+24*frame](i,3);
		lV1.Normalize();
		lV2.Normalize();
		lJoints(3) = -acos(lV1.Dot(lV2));
		mKinRightLeg->setJoints(lJoints.Array());

		mKinRightLeg->getLinkTMatrix(3, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[20+4-1+24*frame](i,3) - lT0(i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1;
		lJoints(4) = GetZRotation(lV1, AXIS_Y);
		mKinRightLeg->setJoints(lJoints.Array());

		mKinRightLeg->getLinkTMatrix(4, lT0);
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[20+4-1+24*frame](i,3) - lT0(i,3);
		lT0.InverseTransformation(lT1);
		lV1 = lT1.GetOrientation()*lV1;
		lJoints(5) = GetZRotation(lV1, AXIS_X);
		mKinRightLeg->setJoints(lJoints.Array());

		mKinRightLeg->getLinkTMatrix(6, lT0);
		for(int i=0; i<3; i++) lV1(i) = -mSkeletonMatrix[20+4-1+24*frame](i,1);
		for(int i=0; i<3; i++) lV2(i) =  lT0(i,2);
		lV1.Normalize();
		lV2.Normalize();
		lJoints(6) = acos(lV1.Dot(lV2));

		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[20+4-1+24*frame](i,2);
		lV1.Normalize();
		if(lV1.Dot(lV2) > 0) lJoints(6) = -lJoints(6);

		mJointTrajectoryRightLeg.SetRow( lJoints, frame);
		//cout << frame << " " << mNbFrame << " " << lError << endl;
	}
}

void SolveIKLeftArm(void)
{
	MathLib::Vector lJoints(DOF_LEFTCHAIN);
	MathLib::Vector lOldJoints(DOF_LEFTCHAIN);
	MathLib::Vector lTarget(IK_CON_ARM);
	MathLib::Vector lTargetVel(IK_CON_ARM);
	MathLib::Vector lCurrent(IK_CON_ARM);
	MathLib::Matrix lJacobianPos(3,DOF_LEFTCHAIN);
	MathLib::Matrix lJacobianDirX(3,DOF_LEFTCHAIN);

	MathLib::Vector lJointsMain(DOF_MAINCHAIN);
	MathLib::Vector lPos(3);
	MathLib::Matrix4 lT;

	int lDirAxis = 1;

	double lError;

	mIKSolverLeft.SetLimits(mJointVelLimitsDn, mJointVelLimitsUp);

	lJoints.Zero();
	mKinLeftArm->setJoints(lJoints.Array());

	int lIndex[4];
	lIndex[0] = 1;
	lIndex[1] = 4;
	lIndex[2] = 5;
	lIndex[3] = 9;

	lJoints.Zero();
	for(int frame=0; frame<mNbFrame; frame++)
	{
		lJointsMain.Set(mJointTrajectoryMain.GetRow(frame));
		lJointsMain(6) = 0.0;
		mKinMain->setJoints(lJointsMain.Array());
		mKinMain->getLinkTMatrix(6, lT);

		mKinLeftArm->setT0(lT);
		mKinLeftArm->readyForKinematics();

		MathLib::Vector3 lV;
		for(int index=0; index<4; index++){
			lV = mSkeletonMatrix[frame*24+7+index].GetTranslation()-mSkeletonMatrix[frame*24+7-1+index].GetTranslation();
			lV = lV/lV.Norm()*mLengthLeft[index] + mSkeletonMatrix[frame*24+7-1+index].GetTranslation();
			mSkeletonMatrix[frame*24+7+index].SetTranslation(lV);
		}

		lTarget.Zero();
		for(int index=0; index<4; index++){
			for(int i=0; i<3; i++) lTarget(i +index*3) = mSkeletonMatrix[index+7+24*frame](i,3); // position
		}
		for(int i=0; i<3; i++) lTarget(i + 12) = -mSkeletonMatrix[10+24*frame](i,0);

		mKinLeftArm->setJoints(lJoints.Array());

		double lOldError=100.0;
		for(int itr=0; itr<50; itr++)
		{
			lCurrent.Zero();
			for(int index=0; index<4; index++){
				mKinLeftArm->getEndPos(lIndex[index], lPos.Array());
				for(int i=0; i<3; i++) lCurrent(i +index*3+ 0) = lPos(i);
			}
			mKinLeftArm->getLinkTMatrix( lIndex[3], lT);
			for(int i=0; i<3; i++) lCurrent(i +12) = lT(i,lDirAxis);

			lError = (lTarget-lCurrent).Norm();


			if( (lError < 0.00001) || (lError> lOldError) )
			{
				if(lError> lOldError)
				{
					lJoints.Set(lOldJoints);
					lError = lOldError;
				}
				break;
			}
			else
			{
				lOldError = lError;
			}

			for(int index=0; index<4; index++){
				mKinLeftArm->getJacobianPos(lIndex[index], lJacobianPos);
				for(int i=0; i<3; i++) mJacobian.SetRow(lJacobianPos.GetRow( i), i +index*3);
			}
			mKinLeftArm->getJacobianDirection(lIndex[3], lDirAxis, lJacobianDirX);
			for(int i=0; i<3; i++) mJacobian.SetRow(lJacobianDirX.GetRow(i), i +12);


			mIKSolverLeft.SetJacobian(mJacobian, 0);
	    	lTargetVel = (lTarget - lCurrent)/DT;

	    	mIKSolverLeft.SetTarget(lTargetVel, 0);
			mIKSolverLeft.Solve();

			lOldJoints.Set(lJoints);
			lJoints += mIKSolverLeft.GetOutput()*DT;

			mKinLeftArm->setJoints(lJoints.Array());
			mKinLeftArm->getJoints(lJoints.Array());
		}
		mJointTrajectoryLeft.SetRow( lJoints, frame);
		cout << frame << " " << mNbFrame << " " << lError << endl;
	}
}

void SolveIKRightArm(void)
{
	MathLib::Vector lJoints(DOF_RIGHTCHAIN);
	MathLib::Vector lOldJoints(DOF_RIGHTCHAIN);
	MathLib::Vector lTarget(IK_CON_ARM);
	MathLib::Vector lTargetVel(IK_CON_ARM);
	MathLib::Vector lCurrent(IK_CON_ARM);
	MathLib::Matrix lJacobianPos(3,DOF_RIGHTCHAIN);
	MathLib::Matrix lJacobianDirX(3,DOF_RIGHTCHAIN);

	MathLib::Vector lJointsMain(DOF_MAINCHAIN);
	MathLib::Vector lPos(3);
	MathLib::Vector lCheckLen(3);
	MathLib::Matrix4 lT;

	int lDirAxis = 1;

	double lError;

	mIKSolverRight.SetLimits(mJointVelLimitsDn, mJointVelLimitsUp);

	lJoints.Zero();
	mKinRightArm->setJoints(lJoints.Array());

	int lIndex[4];
	lIndex[0] = 1;
	lIndex[1] = 4;
	lIndex[2] = 5;
	lIndex[3] = 9;


	for(int frame=0; frame<mNbFrame; frame++)
	{
		lJointsMain.Set(mJointTrajectoryMain.GetRow(frame));
		lJointsMain(6) = 0.0;
		mKinMain->setJoints(lJointsMain.Array());
		mKinMain->getLinkTMatrix(6, lT);

		mKinRightArm->setT0(lT);
		mKinRightArm->readyForKinematics();

		/*
		MathLib::Vector3 lV;
		for(int index=0; index<4; index++){
			lV = mSkeletonMatrix[frame*24+12+index].GetTranslation()-mSkeletonMatrix[frame*24+12-1+index].GetTranslation();
			lV = lV/lV.Norm()*mLengthRight[index] + mSkeletonMatrix[frame*24+12-1+index].GetTranslation();
			mSkeletonMatrix[frame*24+12+index].SetTranslation(lV);
		}
		*/

		lTarget.Zero();
		for(int index=0; index<4; index++){
			for(int i=0; i<3; i++) lTarget(i +index*3) = mSkeletonMatrix[index+12+24*frame](i,3); // position
		}
		for(int i=0; i<3; i++) lTarget(i + 12) = -mSkeletonMatrix[15+24*frame](i,0);

		lJoints.Zero();
		mKinRightArm->setJoints(lJoints.Array());

		double lOldError=100.0;
		for(int itr=0; itr<50; itr++)
		{
			lCurrent.Zero();
			for(int index=0; index<4; index++){
				mKinRightArm->getEndPos(lIndex[index], lPos.Array());
				for(int i=0; i<3; i++) lCurrent(i +index*3+ 0) = lPos(i);
			}
			mKinRightArm->getLinkTMatrix( lIndex[3], lT);
			for(int i=0; i<3; i++) lCurrent(i +12) = lT(i,lDirAxis);

			lError = (lTarget-lCurrent).Norm();

			if( (lError < 0.00001) || (lError> lOldError) )
			{
				if(lError> lOldError)
				{
					lJoints.Set(lOldJoints);
					lError = lOldError;
				}
				break;
			}
			else
			{
				lOldError = lError;
			}

			for(int index=0; index<4; index++){
				mKinRightArm->getJacobianPos(lIndex[index], lJacobianPos);
				for(int i=0; i<3; i++) mJacobian.SetRow(lJacobianPos.GetRow( i), i +index*3);
			}
			mKinRightArm->getJacobianDirection(lIndex[3], lDirAxis, lJacobianDirX);
			for(int i=0; i<3; i++) mJacobian.SetRow(lJacobianDirX.GetRow(i), i +12);

			mIKSolverRight.SetJacobian(mJacobian, 0);
			lTargetVel = (lTarget - lCurrent)/DT;
			//for(int i=0; i<3; i++) lTargetVel(i + 12) *= 0.1;

			mIKSolverRight.SetTarget(lTargetVel, 0);
			mIKSolverRight.Solve();

			lOldJoints.Set(lJoints);
			lJoints += mIKSolverRight.GetOutput()*DT;

			mKinRightArm->setJoints(lJoints.Array());
			mKinRightArm->getJoints(lJoints.Array());
		}
		mJointTrajectoryRight.SetRow( lJoints, frame);
		//cout << frame << " " << mNbFrame << " " << lError << endl;
	}
}


void SolveIKMainManual(void)
{
	MathLib::Vector lJoints(DOF_MAINCHAIN);
	MathLib::Vector lPos(3);
	MathLib::Matrix4 lT0, lT1;
	MathLib::Vector3 lV1, lV2, lV3;

	int lDirAxis = 1;
	double lError;

	lJoints.Zero();
	mKinMain->setJoints(lJoints.Array());


	for(int frame=0; frame<mNbFrame; frame++)
	{
		lJoints.Zero();
		for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[1+24*frame](i,0);
		lV1.Normalize();
		lJoints(0) = GetZRotation(lV1, AXIS_X);
		mKinMain->setJoints(lJoints.Array());

		for(int itr=0; itr<=3; itr++)
		{
			mKinMain->getLinkTMatrix(0+itr*3, lT0);
			for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[2+itr+24*frame](i,3);
			lT0.InverseTransformation(lT1);
			lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
			lJoints(1+itr*3) = GetZRotation(lV1, AXIS_Y);
			mKinMain->setJoints(lJoints.Array());

			mKinMain->getLinkTMatrix(1+itr*3, lT0);
			for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[2+itr+24*frame](i,3);
			lT0.InverseTransformation(lT1);
			lV1 = lT1.GetOrientation()*lV1 + lT1.GetTranslation();
			lJoints(2+itr*3) = GetZRotation(lV1, AXIS_X);
			mKinMain->setJoints(lJoints.Array());

			if( itr == 3 ) break;

			mKinMain->getLinkTMatrix(2+itr*3, lT0);
			for(int i=0; i<3; i++) lV1(i) = mSkeletonMatrix[2+itr+24*frame](i,0);
			lT0.InverseTransformation(lT1);
			lV1 = lT1.GetOrientation()*lV1;
			lJoints(3+itr*3) = GetZRotation(lV1, AXIS_X);
			mKinMain->setJoints(lJoints.Array());
		}

		mJointTrajectoryMain.SetRow( lJoints, frame);
		//cout << frame << " " << mNbFrame << " " << lError << endl;
	}
}

void SolveIKMain(void)
{
	MathLib::Vector lJoints(DOF_MAINCHAIN);
	MathLib::Vector lJointsMain(DOF_MAINCHAIN);
	MathLib::Vector lOldJoints(DOF_MAINCHAIN);
	MathLib::Vector lTarget(IK_CON);
	MathLib::Vector lTargetVel(IK_CON);
	MathLib::Vector lCurrent(IK_CON);
	MathLib::Matrix lJacobianPos(3,DOF_MAINCHAIN);
	MathLib::Matrix lJacobianDirY(3,DOF_MAINCHAIN);

	MathLib::Vector lPos(3);

	double lError;

	mIKSolverMain.SetLimits(mJointVelLimitsDn, mJointVelLimitsUp);

	int lIndex[4];
	lIndex[0] = 2;
	lIndex[1] = 5;
	lIndex[2] = 8;
	lIndex[3] =11;

	MathLib::Matrix4 lT;
	lJoints.Zero();
	for(int frame=0; frame<mNbFrame; frame++)
	{
		lTarget.Zero();
		for(int index=0; index<4; index++){
			for(int i=0; i<3; i++) lTarget(i +index*6+ 0) = mSkeletonMatrix[index+2+24*frame](i,3); // position
			for(int i=0; i<3; i++) lTarget(i +index*6+ 3) = mSkeletonMatrix[index+1+24*frame](i,1); // direction y
		}


		mKinMain->setJoints(lJoints.Array());
		double lOldError=100.0;
		for(int itr=0; itr<50; itr++)
		{
			lCurrent.Zero();
			for(int index=0; index<4; index++){
				mKinMain->getEndPos(lIndex[index]+1, lPos.Array());
				mKinMain->getLinkTMatrix( lIndex[index], lT);
				for(int i=0; i<3; i++) lCurrent(i +index*6+ 0) = lPos(i);
				for(int i=0; i<3; i++) lCurrent(i +index*6+ 3) = lT(i,1);
			}
			lError = (lTarget-lCurrent).Norm();
			//cout << frame << " " << lError<<endl;

			if( (lError < 0.00001) || (lError> lOldError) )
			{
				if(lError> lOldError)
				{
					lJoints.Set(lOldJoints);
					lError = lOldError;
				}
				break;
			}
			else
			{
				lOldError = lError;
			}

			for(int index=0; index<4; index++){
				mKinMain->getJacobianPos(lIndex[index]+1, lJacobianPos);
				mKinMain->getJacobianDirection(lIndex[index], 1, lJacobianDirY);

				for(int i=0; i<3; i++) mJacobian.SetRow(lJacobianPos.GetRow( i), i +index*6 + 0);
				for(int i=0; i<3; i++) mJacobian.SetRow(lJacobianDirY.GetRow(i), i +index*6 + 3);
			}

			mIKSolverMain.SetJacobian(mJacobian, 0);

	    	for(int i=0; i<4; i++)
	    	{
	    		lTargetVel(i*6+0) = (lTarget(i*6+0) - lCurrent(i*6+0) )/DT;
	    		lTargetVel(i*6+1) = (lTarget(i*6+1) - lCurrent(i*6+1) )/DT;
	    		lTargetVel(i*6+2) = (lTarget(i*6+2) - lCurrent(i*6+2) )/DT;

	    		lTargetVel(i*6+3) = (lTarget(i*6+3) - lCurrent(i*6+3) )/DT*0.3;
	    		lTargetVel(i*6+4) = (lTarget(i*6+4) - lCurrent(i*6+4) )/DT*0.3;
	    		lTargetVel(i*6+5) = (lTarget(i*6+5) - lCurrent(i*6+5) )/DT*0.3;
	    	}

	    	mIKSolverMain.SetTarget(lTargetVel, 0);
	    	mIKSolverMain.Solve();

			lOldJoints.Set(lJoints);
			lJoints += mIKSolverMain.GetOutput()*DT;

			mKinMain->setJoints(lJoints.Array());
			mKinMain->getJoints(lJoints.Array());
		}

		mJointTrajectoryMain.SetRow( lJoints, frame);
	}

}

void linkLengthCheck(void)
{
	FILE *fid;
	fid = fopen("./data/linklength.txt", "w+");

	for(int frame=0; frame<mNbFrame; frame++)
	{
		mLengthMain[0]     = (mSkeletonMatrix[ 1  +frame*24].GetTranslation()-mSkeletonMatrix[ 0  +frame*24].GetTranslation()).Norm() ;
		mLengthMain[1]     = (mSkeletonMatrix[ 2  +frame*24].GetTranslation()-mSkeletonMatrix[ 1  +frame*24].GetTranslation()).Norm() ;
		mLengthMain[2]     = (mSkeletonMatrix[ 3  +frame*24].GetTranslation()-mSkeletonMatrix[ 2  +frame*24].GetTranslation()).Norm() ;
		mLengthMain[3]     = (mSkeletonMatrix[ 4  +frame*24].GetTranslation()-mSkeletonMatrix[ 3  +frame*24].GetTranslation()).Norm() ;
		mLengthMain[4]     = (mSkeletonMatrix[ 5  +frame*24].GetTranslation()-mSkeletonMatrix[ 4  +frame*24].GetTranslation()).Norm() ;
		mLengthLeft[0]     = (mSkeletonMatrix[ 8-1+frame*24].GetTranslation()-mSkeletonMatrix[ 7-1+frame*24].GetTranslation()).Norm() ;
		mLengthLeft[1]     = (mSkeletonMatrix[ 9-1+frame*24].GetTranslation()-mSkeletonMatrix[ 8-1+frame*24].GetTranslation()).Norm() ;
		mLengthLeft[2]     = (mSkeletonMatrix[10-1+frame*24].GetTranslation()-mSkeletonMatrix[ 9-1+frame*24].GetTranslation()).Norm() ;
		mLengthLeft[3]     = (mSkeletonMatrix[11-1+frame*24].GetTranslation()-mSkeletonMatrix[10-1+frame*24].GetTranslation()).Norm() ;
		mLengthRight[0]    = (mSkeletonMatrix[13-1+frame*24].GetTranslation()-mSkeletonMatrix[12-1+frame*24].GetTranslation()).Norm() ;
		mLengthRight[1]    = (mSkeletonMatrix[14-1+frame*24].GetTranslation()-mSkeletonMatrix[13-1+frame*24].GetTranslation()).Norm() ;
		mLengthRight[2]    = (mSkeletonMatrix[15-1+frame*24].GetTranslation()-mSkeletonMatrix[14-1+frame*24].GetTranslation()).Norm() ;
		mLengthRight[3]    = (mSkeletonMatrix[16-1+frame*24].GetTranslation()-mSkeletonMatrix[15-1+frame*24].GetTranslation()).Norm() ;
		mLengthLeftLeg[0]  = (mSkeletonMatrix[17-1+frame*24].GetTranslation()-mSkeletonMatrix[   0+frame*24].GetTranslation()).Norm() ;
		mLengthLeftLeg[1]  = (mSkeletonMatrix[18-1+frame*24].GetTranslation()-mSkeletonMatrix[17-1+frame*24].GetTranslation()).Norm() ;
		mLengthLeftLeg[2]  = (mSkeletonMatrix[19-1+frame*24].GetTranslation()-mSkeletonMatrix[18-1+frame*24].GetTranslation()).Norm() ;
		mLengthLeftLeg[3]  = (mSkeletonMatrix[20-1+frame*24].GetTranslation()-mSkeletonMatrix[19-1+frame*24].GetTranslation()).Norm() ;
		mLengthRightLeg[0] = (mSkeletonMatrix[21-1+frame*24].GetTranslation()-mSkeletonMatrix[   0+frame*24].GetTranslation()).Norm() ;
		mLengthRightLeg[1] = (mSkeletonMatrix[22-1+frame*24].GetTranslation()-mSkeletonMatrix[21-1+frame*24].GetTranslation()).Norm() ;
		mLengthRightLeg[2] = (mSkeletonMatrix[23-1+frame*24].GetTranslation()-mSkeletonMatrix[22-1+frame*24].GetTranslation()).Norm() ;
		mLengthRightLeg[3] = (mSkeletonMatrix[24-1+frame*24].GetTranslation()-mSkeletonMatrix[23-1+frame*24].GetTranslation()).Norm() ;

		for(int i=0; i<5 ; i++) fprintf(fid, "%lf ", mLengthMain[i]    );
		for(int i=0; i<4 ; i++) fprintf(fid, "%lf ", mLengthLeft[i]    );
		for(int i=0; i<4 ; i++) fprintf(fid, "%lf ", mLengthRight[i]   );
		for(int i=0; i<4 ; i++) fprintf(fid, "%lf ", mLengthLeftLeg[i] );
		for(int i=0; i<4 ; i++) fprintf(fid, "%lf ", mLengthRightLeg[i]);
		fprintf(fid, "\n");
	}
	fclose(fid);

}

int main(int argc, char **argv)
{
	LoadData();
	ConvertCoordinate();
	linkLengthCheck();

	initKinematicsMain();
	initIKSolver(CHAIN_MAIN);
	//SolveIKMain();
	SolveIKMainManual();

	//initKinematicsLeft();
	//initIKSolver(CHAIN_LEFT);
	//SolveIKLeftArm();

	initKinematicsLeft();
	initIKSolver(CHAIN_LEFT);
	SolveIKLeftArmManual();

	initKinematicsRight();
	initIKSolver(CHAIN_RIGHT);
	SolveIKRightArmManual();

	initKinematicsLeftLeg();
	SolveIKLeftLegManual();

	initKinematicsRightLeg();
	SolveIKRightLegManual();

	FILE *fid;

	fid = fopen("./data/catching_joints.txt", "w+");
	for(int frame=0; frame<mNbFrame; frame++)
	{
		for(int i=0; i<DOF_MAINCHAIN    ; i++) fprintf(fid, "%lf ", mJointTrajectoryMain(    frame, i) );
		for(int i=0; i<DOF_LEFTCHAIN    ; i++) fprintf(fid, "%lf ", mJointTrajectoryLeft(    frame, i) );
		for(int i=0; i<DOF_RIGHTCHAIN   ; i++) fprintf(fid, "%lf ", mJointTrajectoryRight(   frame, i) );
		for(int i=0; i<DOF_LEFTLEGCHAIN ; i++) fprintf(fid, "%lf ", mJointTrajectoryLeftLeg( frame, i) );
		for(int i=0; i<DOF_RIGHTLEGCHAIN; i++) fprintf(fid, "%lf ", mJointTrajectoryRightLeg(frame, i) );
		fprintf(fid, "\n");
	}
	fclose(fid);

	fid = fopen("./data/skeleton_length.txt", "w+");
	for(int i=0; i<5 ; i++) fprintf(fid, "%lf ", mLengthMain[i]    );
	for(int i=0; i<4 ; i++) fprintf(fid, "%lf ", mLengthLeft[i]    );
	for(int i=0; i<4 ; i++) fprintf(fid, "%lf ", mLengthRight[i]   );
	for(int i=0; i<4 ; i++) fprintf(fid, "%lf ", mLengthLeftLeg[i] );
	for(int i=0; i<4 ; i++) fprintf(fid, "%lf ", mLengthRightLeg[i]);
	fprintf(fid, "\n");
	fclose(fid);

	return 0;
}
