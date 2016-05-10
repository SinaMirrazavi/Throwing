/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef RandomThrowing_H_
#define RandomThrowing_H_

#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"

#include "RobotInterface.h"
#include "MathLib.h"
#include "IKGroupSolver.h"

#include "ForwardDynamics.h"
#include "InverseDynamics.h"
#include "KinematicChain.h"
#include "PIDController.h"

#include "Gaussians.h"
#include "CDDynamics.h"
#include "ThirdPoly.h"
#include "sKinematics.h"
#include "OptiTrack.h"

#include "Ballistic.h"

enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};
enum ENUM_STANDBY{STANDBY_OFF, STANDBY_ON};
enum ENUM_COMMAND{COMMAND_REST, COMMAND_JOB, COMMAND_HOME, COMMAND_THROWJOINT_READY, COMMAND_NONE};
enum ENUM_HANDCOMMAND{HANDCOMMAND_GRASP, HANDCOMMAND_RELEASE, HANDCOMMAND_HOME, HANDCOMMAND_NONE};
enum ENUM_STATUS{STATUS_IDLE_PRETHROW, STATUS_THROW_READY, STATUS_THROW_THROWING, STATUS_THROW_REST,
	STATUS_THROWJOINT_READY, STATUS_THROWJOINT_THROWING, STATUS_THROWJOINT_REST,STATUS_REST, STATUS_NONE};
enum ENUM_PLANNER{PLANNER_CDDYN, PLANNER_POLY};

#define WN_POSITION_FILTER 50
#define WN_JOINT_PLANNER 3
#define WN_JOINT_FILTER 25
#define WN_JOINT_FILTER_STRONG 370
#define WN_FINGER_PLANNER 15
#define WN_FINGER_PLANNER_STRONG 30
#define WN_THROW_PLANNER 7

#define RECORDING_TIME 10.0

#define _dt (1.0/500.)
#define KUKA_DOF 7
#define KUKA_FINGER_DOF 16
#define KUKA_CATCHING_DIR AXIS_X
#define IK_CONSTRAINTS 9

#define TARGET_CONT_FRAME 1


double cJob[]    = {DEG2RAD(90.), DEG2RAD(60.0), DEG2RAD(45.0+10.0), DEG2RAD(-23.0), DEG2RAD(90.0+45.), DEG2RAD(-45.0000), DEG2RAD(0.0)};
double cThrow[]  = {DEG2RAD(90.), DEG2RAD(60.0), DEG2RAD(45.0+10.0), DEG2RAD(-23.0), DEG2RAD(90.0+45.), DEG2RAD(-45.0000), DEG2RAD(40.0)};


// for ball
double cHandGrasp[]   = {0.150, 0.20, 0.9, 0.0,  -0.15, 0.20, 0.9, 0.00,  0.0,0.0,0.0,0.0,  1.3, 0.2, 1.5, 0.0};
double cHandRelease[] = {0.150, 0.00, 0.9, 0.0,  -0.15, 0.00, 0.9, 0.00,  0.0,0.0,0.0,0.0,  1.3, 0.2, 0.2, 0.0};
//double cHandGrasp[]  ={0.28, 0.55, 0.54, 0.0,  0.00, 0.010, 1.25, 0.00,  -0.28, 0.55, 0.54, 0.00,  1.35, 0.35, 1.30, 0.20};
//double cHandRelease[]={0.28, 0.55, 0.54, 0.0,  0.00, 0.010, 1.25, 0.00,  -0.28, 0.55, 0.54, 0.00,  1.35, 0.35, 0.20, 0.00};

// for bottle
//double cHandGrasp[]  ={0.0, 0.0, 1.2, 0.35,  0.00, 0.00, 1.2, 0.35,  0.0, 0.00, 1.2, 0.35,  1.35, 0.35, 1.30, 0.20};
//double cHandRelease[]={0.0, 0.0, 1.2, 0.35,  0.00, 0.00, 1.2, 0.35,  0.0, 0.00, 1.2, 0.35,  1.35, 0.35, 0.20, 0.00};

double cHandHome[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DEG2RAD(70.0),  DEG2RAD(45.0), 0.0, 00.0};

class RandomThrowing : public RobotInterface
{
protected:
	ENUM_COMMAND mCommand;
	ENUM_HANDCOMMAND mHandCommand;
	ENUM_STATUS  mStatus;
	ENUM_PLANNER  mPlanner;
	ENUM_STANDBY mStanby;

	// ros node
	ros::NodeHandle *nodeNetwork;

	// finger
	sensor_msgs::JointState mMsgJoint;
	std_msgs::Int16 mMsgThrowing;

	// kinematics
    int                         mEndEffectorId;
    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;
    KinematicChain              mKinematicChain;
    sKinematics                 *mSKinematicChain;

    IKGroupSolver               mIKSolver;
    Matrix                      mJacobianPos;
    Matrix                      mJacobianDir;
    Matrix                      mJacobian9;
	Vector                      mJointPos;
	Vector                      mJointFinger;
	Vector                      mJointWeights;
	Vector                      mJointAll;

	Vector                      mJob;

	// joint planner
    Vector      mJointVelLimitsUp;
    Vector      mJointVelLimitsDn;
	CDDynamics *mCDJointPlanner;
	CDDynamics *mCDJointFilter;

	Vector mJointVelocityLimitsDT;

	ThirdPoly *mPolyJointPlanner;
	CDDynamics *mThrowPlanner;

	// target
	Vector mJointReady;
	Vector mJointRest;
	Vector mJointThrow;
	Vector mThrowDir;
	double mThrowMag;

	Matrix mThrowingJointTrj;

	//
	Ballistic *mBallistic;
	double mOffsetYaw;
	double mOffsetMag;

	// log
	int mRecordingIndex;
	MathLib::Matrix mRobotTrj;
	//Skeleton mSkeletonTrj[500*RECORDING_TIME];
	Skeleton mSkeleton;
	MathLib::Matrix mThrowingPos;
	int mThrowingOrder[45];

	void initKinematics(void);
	void syncRobotChain(void);

	void initThrowingPos(void);

	void calculateThrowJointTrajectory(Vector3& targetPos);
	void _calculateThrowJointTrajectory(void);
public:
            RandomThrowing();
    virtual ~RandomThrowing();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
};



#endif 
