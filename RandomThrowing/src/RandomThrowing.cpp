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

#include "RandomThrowing.h"
#include "SkeletonStreaming.h"


#define JOINT_STATE_TOPIC "/allegro/joint_state"
#define JOINT_CMD_TOPIC "/allegro/joint_cmd"

CDDynamics *gFingerPlanner;
ros::Subscriber sub_handJoint;
ros::Publisher pub_handJoint;

ros::Subscriber sub_skeleton;
ros::Publisher pub_skeleton;

MathLib::Vector3 gShoulderPos;
double gArmLength;
double gFingerPos[16];

int gIdleIndex=0;

bool gIsPlannerINIT = false;
int mIndex = 0;

void SkeletonInfoCallback(const geometry_msgs::PoseStamped& msg)
{
	gShoulderPos(0) = msg.pose.position.x;
	gShoulderPos(1) = msg.pose.position.y;
	gShoulderPos(2) = msg.pose.position.z;

	if( mIndex == 0 ){
		gArmLength = msg.pose.orientation.w*0.9;
	}
	/*
	gShoulderPos(0) = -3.0;
	gShoulderPos(1) = -0.1;
	gShoulderPos(2) =  0.2;
	*/
	//gArmLength = 0.6*0.85;
}

void FingerJointCallback(const sensor_msgs::JointState& msg)
{
	MathLib::Vector pos(16), target(16);

	if( gIsPlannerINIT == false){
		for(int j=0; j<16; j++) pos(j) = msg.position[j];
		gFingerPlanner->SetStateTarget(pos, pos);

		gIsPlannerINIT = true;
	}
	else
	{
		for(int j=0; j<16; j++) gFingerPos[j] = msg.position[j];
	}
}

void RandomThrowing::initThrowingPos()
{
	mThrowingPos.Resize(21, 2);

	gShoulderPos(0) = -3.0;
	gShoulderPos(1) = -0.1;
	gShoulderPos(2) =  0.2;
	gArmLength = 0.6*0.85;

	int index=0;
	for( int y=0; y<3; y++)
	{
		for( int z=0; z<7; z++)
		{
			mThrowingPos(index,0) = ((double)y-1.0)/1.0*0.5;
			mThrowingPos(index,1) = ((double)z-4.0)/3.0*1.0;
			//printf("%lf %lf \n", mThrowingPos(index,0), mThrowingPos(index,1));
			index++;
		}
	}
	int randPerm[]= {11, 19,1, 13, 14, 9,12,17,0,7,20, 8, 4,16,10, 6, 5,15, 2, 3, 18};
	for(int i=0; i<21; i++) mThrowingOrder[i] = randPerm[i];
}


RandomThrowing::RandomThrowing()
:RobotInterface(){
}
RandomThrowing::~RandomThrowing(){
}


void RandomThrowing::initKinematics(void){
	mSKinematicChain = new sKinematics(KUKA_DOF, _dt);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	mSKinematicChain->setDH(0,   0.0,    0.310,       M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(132.0)*0.95);
	mSKinematicChain->setDH(1,   0.0,    0.000,      -M_PI_2, 0.0, 1,  DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(132.0)*0.95);
	mSKinematicChain->setDH(2,   0.0,    0.400,      -M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(128.0)*0.95);
	mSKinematicChain->setDH(3,   0.0,    0.000,       M_PI_2, 0.0, 1,  DEG2RAD(-100.), DEG2RAD(100.), DEG2RAD(128.0)*0.95);
	mSKinematicChain->setDH(4,   0.0,    0.390,       M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(204.0)*0.95);
	mSKinematicChain->setDH(5,   0.0,    0.000,      -M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(184.0)*0.95); // reduced joint angle to save the fingers
	mSKinematicChain->setDH(6,  -0.04,  0.117+0.18,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(184.0)*0.95);

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
	mJacobianPos.Resize(3,KUKA_DOF);
	mJacobianDir.Resize(3,KUKA_DOF);
	mJacobian9.Resize(9,KUKA_DOF);

	mJointPos.Resize(KUKA_DOF);
	mJointWeights.Resize(KUKA_DOF);

}

void RandomThrowing::syncRobotChain(void)
{
    Vector3 lPos, lDir;

	// read from sensor
    mSensorsGroup.ReadSensors();
    mJointAll = mSensorsGroup.GetJointAngles();
    mJointPos.Set( mJointAll.Array(), KUKA_DOF);

    mSKinematicChain->setJoints(mJointPos.Array());

    mSKinematicChain->getEndPos(lPos.Array());

    // init joint filer
    mCDJointPlanner->SetStateTarget(mJointPos, mJointPos);
    mCDJointFilter->SetStateTarget(mJointPos, mJointPos);
}

RobotInterface::Status RandomThrowing::RobotInit(){
	// initialize sensor group
	mSensorsGroup.SetSensorsList(mRobot->GetSensors());
	mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());
	mSensorsGroup.ReadSensors();
	mJointAll.Resize(KUKA_DOF+KUKA_FINGER_DOF);

	// Finger
	mJointFinger.Resize(KUKA_FINGER_DOF);
	gFingerPlanner = new CDDynamics(KUKA_FINGER_DOF, _dt, WN_FINGER_PLANNER );

	// Kinematic Chain for real robot
	mEndEffectorId = mRobot->GetLinksCount()-1;
	mKinematicChain.SetRobot(mRobot);
	mKinematicChain.Create(0,0,mEndEffectorId);

	// Kinematic chain for virtual robot
	initKinematics();
	mJob.Resize(KUKA_DOF);
	mJob.Set(cJob, KUKA_DOF);

	// Inverse Kinematics
	mIKSolver.SetSizes(KUKA_DOF);
	mIKSolver.AddSolverItem(3);                 // One solver with 6 constraints
	mIKSolver.SetVerbose(false);                // No comments
	mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
	mIKSolver.Enable(true,0);                   // Enable first solver
	mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver

	//
	mPolyJointPlanner = new ThirdPoly(KUKA_DOF);
	mThrowPlanner = new CDDynamics(KUKA_DOF, _dt, WN_JOINT_PLANNER);

	//initializeTracker();

	// target variable resize
	mJointReady.Resize(KUKA_DOF);
	mJointRest.Resize(KUKA_DOF);
	mJointThrow.Resize(KUKA_DOF);
	mThrowDir.Resize(3);

	mThrowingJointTrj.Resize(500+TARGET_CONT_FRAME+1000,KUKA_DOF);
	initThrowingPos();

	double maxVel[7];
	mSKinematicChain->getMaxVel(maxVel);

	mJointVelLimitsUp.Resize(KUKA_DOF);
	mJointVelLimitsDn.Resize(KUKA_DOF);
	mJointVelocityLimitsDT.Resize(KUKA_DOF);
	mJointVelocityLimitsDT.Set(maxVel, KUKA_DOF);
	mJointVelocityLimitsDT *= _dt;

	mCDJointPlanner   = new CDDynamics(KUKA_DOF, _dt, WN_JOINT_PLANNER  );
	mCDJointFilter    = new CDDynamics(KUKA_DOF, _dt, WN_JOINT_FILTER   );

	mBallistic = new Ballistic(_dt);
	mOffsetYaw = DEG2RAD(4.1);
	mOffsetMag = 1.0;

	mRecordingIndex = 0;
	//mRobotTrj.Resize(500*RECORDING_TIME, 7+12);

	mCommand = COMMAND_NONE;
	mHandCommand = HANDCOMMAND_NONE;
	mStatus  = STATUS_NONE;
	mStanby = STANDBY_OFF;

	AddConsoleCommand("test");
	AddConsoleCommand("job");
	AddConsoleCommand("home");
	AddConsoleCommand("throw");
	AddConsoleCommand("standby");
	AddConsoleCommand("move");
	AddConsoleCommand("grasp");
	AddConsoleCommand("setym");
	AddConsoleCommand("getym");
	AddConsoleCommand("cal");

	return STATUS_OK;
}
RobotInterface::Status RandomThrowing::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status RandomThrowing::RobotStart(){
	nodeNetwork = mRobot->InitializeROS();
	mMsgJoint.position.resize(4*4);
	mMsgJoint.velocity.resize(4*4);
	sub_handJoint = nodeNetwork->subscribe(JOINT_STATE_TOPIC, 3, FingerJointCallback);
	pub_handJoint = nodeNetwork->advertise<sensor_msgs::JointState>(JOINT_CMD_TOPIC, 3);

	sub_skeleton = nodeNetwork->subscribe(THROWING_STATE_TOPIC, 3, SkeletonInfoCallback);
	pub_skeleton = nodeNetwork->advertise<std_msgs::Int16>(THROWING_CMD_TOPIC, 1);

    return STATUS_OK;
}    
RobotInterface::Status RandomThrowing::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status RandomThrowing::RobotUpdate(){
	static int lCnt = 0;
	static int lThrowingIndex=0;
	Vector ljoints(KUKA_DOF);
	Vector lPos(3);

	if( lCnt%100 == 0)
	{
		mMsgThrowing.data = VISIONCOMMAND_SKELETONINFO;
		pub_skeleton.publish(mMsgThrowing);
	}
	lCnt++;

	// command
	switch(mCommand){
        case COMMAND_REST:
        	syncRobotChain();

        	mPlanner = PLANNER_CDDYN;
        	mStatus  = STATUS_REST;
        	break;
        case COMMAND_JOB:
        	syncRobotChain();
        	mCDJointPlanner->SetTarget(mJob);

        	mPlanner = PLANNER_CDDYN;
        	mStatus  = STATUS_REST;

    		mSKinematicChain->getEndPos(lPos.Array());

        	break;
        case COMMAND_THROWJOINT_READY:
        	syncRobotChain();

        	mCDJointPlanner->SetTarget(mJointReady);

        	mPlanner = PLANNER_CDDYN;
        	mStatus  = STATUS_THROWJOINT_READY;
        	break;
        case COMMAND_NONE:

        	break;
        default:
        	syncRobotChain();
        	mPlanner = PLANNER_CDDYN;
    }
	mCommand = COMMAND_NONE;


    // hand command
	switch(mHandCommand){
    case HANDCOMMAND_GRASP:
    	mJointFinger.Set(cHandGrasp, KUKA_FINGER_DOF);
    	gFingerPlanner->SetWn(WN_FINGER_PLANNER);
    	gFingerPlanner->SetTarget(mJointFinger);
    	break;
    case HANDCOMMAND_RELEASE:
    	mJointFinger.Set(cHandRelease, KUKA_FINGER_DOF);
    	gFingerPlanner->SetWn(WN_FINGER_PLANNER_STRONG);
    	gFingerPlanner->SetTarget(mJointFinger);
    	break;
    case HANDCOMMAND_HOME:
    	mJointFinger.Set(cHandHome, KUKA_FINGER_DOF);
    	gFingerPlanner->SetWn(WN_FINGER_PLANNER);
    	gFingerPlanner->SetTarget(mJointFinger);
    	break;
    }
	mHandCommand = HANDCOMMAND_NONE;


	switch(mStatus){
		case STATUS_THROWJOINT_READY:
	//		cout<<"1"<<endl;
			mCDJointPlanner->Update();
			mCDJointPlanner->GetState(ljoints);


			if( (ljoints - mJointReady).Norm() < 0.001 )
			{
				mStatus = STATUS_THROWJOINT_THROWING;
				lThrowingIndex = 0;
				mCDJointFilter->SetWn(WN_JOINT_FILTER_STRONG);
			}
			break;
		case STATUS_THROWJOINT_THROWING:
		//	cout<<"2"<<endl;
			if( lThrowingIndex < 500+TARGET_CONT_FRAME+1000-1 )
			{
				// 400 too early
				//if( lThrowingIndex == 410 )
				if( lThrowingIndex == 460 )
				{
					mHandCommand = HANDCOMMAND_RELEASE;
				}

				ljoints = mThrowingJointTrj.GetRow(lThrowingIndex);
				lThrowingIndex++;
			}
			else if( lThrowingIndex == (500+TARGET_CONT_FRAME+1000-1) )
			{
				ljoints = mThrowingJointTrj.GetRow(lThrowingIndex);

				mCDJointFilter->SetWn(WN_JOINT_FILTER);
				mCDJointPlanner->SetStateTarget(ljoints, ljoints);

				mStatus = STATUS_REST;

		    	mJob.Set(cJob, KUKA_DOF);
		    	mCDJointPlanner->Update();
		    	mCDJointPlanner->SetTarget(mJob);
		    	mPlanner = PLANNER_CDDYN;
			}
			break;
		case STATUS_IDLE_PRETHROW:
		//	cout<<"3"<<endl;
			mCDJointPlanner->Update();
			mCDJointPlanner->GetState(ljoints);

			gIdleIndex--;
			if( gIdleIndex < 0 ) mCommand = COMMAND_THROWJOINT_READY;

			break;
		default:
		//	cout<<"4"<<endl;
			mCDJointPlanner->Update();
			mCDJointPlanner->GetState(ljoints);
	}


	for(int i=0; i<KUKA_DOF; i++){
		if( ljoints(i) >  mSKinematicChain->getMax(i) ) ljoints(i) = mSKinematicChain->getMax(i);
		if( ljoints(i) <  mSKinematicChain->getMin(i) ) ljoints(i) = mSKinematicChain->getMin(i);
	}
	mCDJointFilter->SetTarget(ljoints);

    return STATUS_OK;
}
RobotInterface::Status RandomThrowing::RobotUpdateCore(){
	Vector lFingerPos(KUKA_FINGER_DOF);
	double lOri[3][3];
	Vector3 lPos;
	double lArmLength;
	Vector3 lTargetPos;
	int lOrder;
	char fname[100];

	// get current joint angles from real robot
	mSensorsGroup.ReadSensors();

	mJointAll = mSensorsGroup.GetJointAngles();
	mJointPos.Set( mJointAll.Array(), KUKA_DOF);
	//mJointTorque = mSensorsGroup.GetJointTorques();

	if( mStatus == STATUS_NONE ){
		mCDJointPlanner->SetStateTarget(mJointPos, mJointPos);
		mCDJointFilter->SetStateTarget(mJointPos, mJointPos);
		mStatus = STATUS_REST;
	}

	// finger motion
	if( gIsPlannerINIT == true){
		gFingerPlanner->Update();
		gFingerPlanner->GetState(lFingerPos);

		for(int i=0; i<16; i++){
			mMsgJoint.position[i] = lFingerPos(i);
		}

		pub_handJoint.publish(mMsgJoint);
	}

	// filter joint angles
	mCDJointFilter->Update();
	mCDJointFilter->GetState(mJointPos);

	for(int i=0; i<KUKA_DOF; i++){
		if( mJointPos(i) >  mSKinematicChain->getMax(i) ) mJointPos(i) = mSKinematicChain->getMax(i);
		if( mJointPos(i) <  mSKinematicChain->getMin(i) ) mJointPos(i) = mSKinematicChain->getMin(i);
	}

	if(mRobot->GetControlMode()!=Robot::CTRLMODE_POSITION)
		mRobot->SetControlMode(Robot::CTRLMODE_POSITION);

	// Send command to the robot.
	mActuatorsGroup.SetJointAngles(mJointPos);
	mActuatorsGroup.WriteActuators();
	mKinematicChain.Update();

	// standby
	if( (mStanby == STANDBY_ON) && (mStatus == STATUS_REST) && (mRecordingIndex ==0))
	{
		// if triger
	//	cout<<"fabs(mJointAll(0)-mJointPos(0))  "<<fabs(mJointAll(0)-mJointPos(0)) <<endl;
		if( fabs(mJointAll(0)-mJointPos(0))< DEG2RAD(0.2) )
		{
			if (mJointAll(0) > mJointPos(0)) mIndex++;

			if(mIndex<0) mIndex = 0;
			cout << "mIndex " << mIndex << " " << fabs(mJointAll(0)-mJointPos(0)) << endl;
			lPos(0) = 0.0;
			lPos(1) = mThrowingPos(lOrder, 0) * gArmLength;
			lPos(2) = mThrowingPos(lOrder, 1) * gArmLength;
			lPos.Print("lPos");
			lTargetPos = gShoulderPos + lPos;
			//lTargetPos = gShoulderPos;

			cout << "Shoulder Pos:" << gShoulderPos(0) << " " << gShoulderPos(1) << " " << gShoulderPos(2) << ", lOrder :" << lOrder << " ,length: " << gArmLength<< " " << mThrowingPos(lOrder, 0) << endl;
			cout << "Target   Pos:" << lTargetPos(0) << " " << lTargetPos(1) << " " << lTargetPos(2) << " " << endl;

			if( lTargetPos.Norm() < 0.0001 )
			{
				cout << "please check vision system " << endl;
			}
			else{
				mJointThrow.Set(cThrow, KUKA_DOF);

				mThrowDir(0) = -1.0;
				mThrowDir(1) =  0.0;
				mThrowDir(2) =  1.0;
				mThrowDir.Normalize();
				mThrowMag = 10.0;

				calculateThrowJointTrajectory(lTargetPos);

				mRecordingIndex = 1;

				mMsgThrowing.data = VISIONCOMMAND_STARTRECORDING;
				pub_skeleton.publish(mMsgThrowing);

				mHandCommand = HANDCOMMAND_GRASP;
				gIdleIndex = 500;
				mStatus = STATUS_IDLE_PRETHROW;
				//mCommand = COMMAND_THROWJOINT_READY;
			}
		}
	}

	double lBallPos[3];
	double lEndPos[3];

	if(mRecordingIndex > 0 )
	{
		// get robot end position (desired)
		mSKinematicChain->setJoints(mJointPos.Array());
		mSKinematicChain->getEndPos(lEndPos);

		mRecordingIndex++;

		if( mRecordingIndex >= 500*RECORDING_TIME)
		{
	    	mHandCommand = HANDCOMMAND_RELEASE;
			mRecordingIndex = 0;

	    	mMsgThrowing.data = VISIONCOMMAND_STOPRECORDING;
			pub_skeleton.publish(mMsgThrowing);
			cout << (mIndex) << " throwing done!" << endl;

	    	if( mIndex == 21)
	    	{
	    		mIndex = 0;
	    		mStanby = STANDBY_OFF;
	    		mCommand = COMMAND_REST;
	    	}
		}
	}

	ros::spinOnce();

	return STATUS_OK;
}
int RandomThrowing::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	Vector ljoints(KUKA_DOF);
	Vector3 lTargetPos;

    if(cmd=="rest"){
        mCommand = COMMAND_REST;
    }
    else if(cmd=="home"){
        mCommand = COMMAND_HOME;
    }
    else if(cmd=="job"){
    	mJob.Set(cJob, KUKA_DOF);
    	mHandCommand = HANDCOMMAND_RELEASE;
        mCommand = COMMAND_JOB;
    }
    else if(cmd=="standby")
    {
    	mStanby = STANDBY_ON;
    }
    else if(cmd=="throw"){
    	mHandCommand = HANDCOMMAND_GRASP;

    	mJointThrow.Set(cThrow, KUKA_DOF);

    	mThrowDir(0) = -1.0;
    	mThrowDir(1) =  0.0;
    	mThrowDir(2) =  1.0;
    	mThrowDir.Normalize();
    	mThrowMag = 10.0;

    	mIndex = 0;
    }
    else if(cmd=="move"){
        if(args.size()==KUKA_DOF){
        	for( int i=0; i<KUKA_DOF; i++ ){
        		ljoints(i) =  atof( args[i].c_str() );
        	}
        	mJob.Set(ljoints);
			mCommand = COMMAND_JOB;
        }
        else{
        	cout << args.size() << endl;
        }

    }

    else if(cmd == "grasp")
    {
    	mHandCommand = HANDCOMMAND_GRASP;
    }
    else if(cmd =="setym")
    {
        if(args.size()==2){
        	mOffsetYaw = DEG2RAD( atof( args[0].c_str() ) );
        	mOffsetMag = atof( args[1].c_str() );
        }
        else{
        	cout << "wrong input" << args.size() << endl;
        }
    }
    else if(cmd =="getym")
    {
    	cout << "yaw "<< RAD2DEG(mOffsetYaw) << ", Mag "<< mOffsetMag << endl;
    }
    else if(cmd == "cal")
    {
    	mJointThrow.Set(cThrow, KUKA_DOF);
        if(args.size()==1){
        	mJointThrow(5) = DEG2RAD( atof( args[0].c_str() ) );

        	mSKinematicChain->setJoints(mJointThrow.Array());
        	mSKinematicChain->getEndDirAxis(2, mThrowDir.Array() );
        	mThrowDir.Print();
        }
    	mCDJointPlanner->SetTarget(mJointThrow);
    	mPlanner = PLANNER_CDDYN;
    }
    return 0;
}



void RandomThrowing::calculateThrowJointTrajectory(Vector3& targetPos)
{
	Vector lJoint(KUKA_DOF);
	Vector lJointsVel(KUKA_DOF);
	Vector lInitVel(KUKA_DOF), lTargetVel(KUKA_DOF);
	Vector lPos(3), lVel(3);

	mJointThrow.Set(cThrow, KUKA_DOF);

	lJointsVel(0) = -mSKinematicChain->getMaxVel(0);
	lJointsVel(1) = -mSKinematicChain->getMaxVel(1);
	lJointsVel(2) = -mSKinematicChain->getMaxVel(2);
	lJointsVel(3) =  mSKinematicChain->getMaxVel(3);
	lJointsVel(4) = -mSKinematicChain->getMaxVel(4);
	lJointsVel(5) =  mSKinematicChain->getMaxVel(5);
	lJointsVel(6) =  mSKinematicChain->getMaxVel(6);


	mSKinematicChain->setJoints(mJointThrow.Array());
	mSKinematicChain->getJacobianPos(mJacobianPos);
	mSKinematicChain->getEndPos(lPos.Array());
	lVel = mJacobianPos*lJointsVel;

	// find yaw angle
	Vector lcV(2);
	Vector lcT(2);
	Vector lcP(2);
	Vector lcZero(2);
	Vector3 lError;

	lcV.Set(lVel.Array(), 2);
	lcT.Set(targetPos.Array(), 2);
	lcP.Set(lPos.Array(), 2);

	lcZero(0) = 0.0; lcZero(1) = -1.0;
	double angldefault  = acos( lcZero.Dot(lcP)/(lcZero.Norm()*lcP.Norm()));

	double alpha = M_PI-acos( lcV.Dot(lcP)/(lcV.Norm()*lcP.Norm()) );
	double beta = M_PI-( alpha+asin( lcP.Norm()/lcT.Norm()*sin(alpha)) );
	double gamma;

	lcZero(0) = -1.0; 	lcZero(1) =  0.0;
	if( lcT(1) < 0 )
		gamma = beta + acos( lcZero.Dot(lcT)/(lcZero.Norm()*lcT.Norm())) - angldefault;
	else
		gamma = beta - acos( lcZero.Dot(lcT)/(lcZero.Norm()*lcT.Norm())) - angldefault;

	mJointThrow(0) = gamma;

	//lVel.Print("velocity");
	//lPos.Print("position");
	//printf("%lf \n",  RAD2DEG(gamma) );

	// find magnitude
	mSKinematicChain->setJoints(mJointThrow.Array());
	mSKinematicChain->getJacobianPos(mJacobianPos);
	mSKinematicChain->getEndPos(lPos.Array());
	lVel = mJacobianPos*lJointsVel;

	double lDuration;
	double lKp = 1.0;
	lError.Zero();

	for(int i=0; i<60; i++){
		lKp  *= (1.0 -(lError(0)*0.02));
		lKp  *= (1.0 +(lError(2)*0.02));

		mBallistic->SetPosVel(lPos.Array(), (lVel*lKp).Array());
		lDuration = mBallistic->CalSmallestError(targetPos, lError);

		if( lError.Norm() < 0.001) break;
		//cout << "error : " << lError(0) << " " << lError(1) << " " << lError(2) << ", kp " <<  lKp << endl;
	}

	cout << "error : " << lError(0) << " " << lError(1) << " " << lError(2) << ", kp " <<  lKp << " gamma:" << gamma<< endl;

	mOffsetMag = (0.907-1.04)*(lKp-0.6)/(1.08-0.6) + 1.04;

	lKp *= mOffsetMag;
	// 0.6  1.04
	// 0.7  1.01
	// 0.87 0.955
	// 1.08 0.93

	// gamma
	// 1.0847
	// 0.8435 4.7
	// 0.849 4.4

	if(lKp>1.0)
	{
		cout <<"Cannot throw for that position " << lKp << endl;
		lKp = 1.0;
	}

	lJointsVel *= lKp;
	lVel = mJacobianPos*lJointsVel;
	mJointThrow(0) += mOffsetYaw;

	printf("yaw : %lf, mag : %lf \n", RAD2DEG(gamma), lKp);

	mThrowingJointTrj.SetRow(mJointThrow, 500);

	mJointReady = mJointThrow - (lJointsVel*_dt*70.0);
	mJointReady(4) = mJointThrow(4);
	mJointRest = mJointThrow;

	for(int i=0;i<KUKA_DOF;i++){
		if( mJointReady(i) < mSKinematicChain->getMin(i) + 0.01 ){
			mJointReady(i) = mSKinematicChain->getMin(i) + 0.01;
		}else if( mJointReady(i) > mSKinematicChain->getMax(i) - 0.01 ){
			mJointReady(i) = mSKinematicChain->getMax(i) - 0.01;
		}
	}


	// first generate throwing motion
	lInitVel.Zero();
	lTargetVel = lJointsVel;
	mPolyJointPlanner->SetConstraints(mJointReady, lInitVel, mJointThrow, lTargetVel, 1.0);
	for(int i=0; i<500; i++)
	{
		mPolyJointPlanner->Get((double)(i)*_dt, lJoint);
		mThrowingJointTrj.SetRow(lJoint, i);
	}

	// goto stop motion
	lInitVel = lJointsVel;
	lTargetVel.Zero();
	//mPolyJointPlanner->SetConstraints(mJointThrow, lInitVel, mJointRest, lTargetVel, 1.0);
	mThrowPlanner->SetWn(WN_THROW_PLANNER);
	mThrowPlanner->SetState(mJointThrow, lInitVel);
	mThrowPlanner->SetTarget(mJointRest);
	for(int i=0; i<1000; i++)
	{
		//mPolyJointPlanner->Get((double)(i+1)*_dt, lJoint);
		mThrowPlanner->Update();
		mThrowPlanner->GetState(lJoint);
		mThrowingJointTrj.SetRow(lJoint, i+500+TARGET_CONT_FRAME);
	}

	//mThrowingJointTrj.Save("/home/seungsu/trj.txt", 10, 500+TARGET_CONT_FRAME+1000);
}


void RandomThrowing::_calculateThrowJointTrajectory(void)
{
	Vector lJoint(KUKA_DOF), lJointIN(KUKA_DOF);
	Vector lPos(3);
	double lDuration;

	Vector lNextTargetVel(3);
	Vector lNextTargetPos(3);
	Vector lJointsVel(KUKA_DOF);
	Vector lJointVelRes(KUKA_DOF);
	Matrix lWeight(KUKA_DOF, KUKA_DOF);
	lWeight.Zero();

	Vector lInitJoint(KUKA_DOF);
	Vector lInitVel(KUKA_DOF);
	Vector lTargetVel(KUKA_DOF);


	Vector lJointThrowingStart(KUKA_DOF);
	Vector lJointThrowingEnd(KUKA_DOF);


	// calculate throwing for 0.1 sec (TARGET_CONT_FRAME step)
	lJoint = mJointThrow;
	for(int frame=0; frame<TARGET_CONT_FRAME; frame++)
	{
		mSKinematicChain->setJoints(lJoint.Array());
		mSKinematicChain->getJacobianPos(mJacobianPos);
		mSKinematicChain->getEndPos(lPos.Array());


		for(int i=0;i<KUKA_DOF;i++){
			mJointVelLimitsDn(i) =  mSKinematicChain->getMin(i);
			mJointVelLimitsUp(i) =  mSKinematicChain->getMax(i);
		}
		mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);


		lWeight.Identity();
		lJointsVel.Zero();
		for(int ikrpt =0; ikrpt<15; ikrpt++)
		{
			lNextTargetVel = (mThrowDir*mThrowMag - mJacobianPos*lJointsVel);

			mIKSolver.SetJacobian(mJacobianPos*lWeight);
			mIKSolver.SetTarget(lNextTargetVel, 0);
			mIKSolver.Solve();
			lJointsVel += mIKSolver.GetOutput();

			for(int i=0;i<KUKA_DOF; i++){
				if( lJointsVel(i) >  mSKinematicChain->getMaxVel(i) ){
					lJointsVel(i) = mSKinematicChain->getMaxVel(i);
					lWeight(i,i) = 0.0;
				}
				else if( lJointsVel(i) < -mSKinematicChain->getMaxVel(i) ){
					lJointsVel(i) = -mSKinematicChain->getMaxVel(i);
					lWeight(i,i) = 0.0;
				}
				else{
					lWeight(i,i) = 1.0;
				}
			}
			(mJacobianPos*lJointsVel).Print();
		}
		lJointsVel.Print();

		/*
		lJointsVel(0) = -mSKinematicChain->getMaxVel(0);
		lJointsVel(1) = -mSKinematicChain->getMaxVel(1);
		lJointsVel(2) = -mSKinematicChain->getMaxVel(2);
		lJointsVel(3) =  mSKinematicChain->getMaxVel(3);
		lJointsVel(4) = -mSKinematicChain->getMaxVel(4);
		lJointsVel(5) =  mSKinematicChain->getMaxVel(5);
		lJointsVel(6) =  mSKinematicChain->getMaxVel(6);
		*/
		lJointsVel(0) = -mSKinematicChain->getMaxVel(0);
		lJointsVel(1) = -mSKinematicChain->getMaxVel(1);
		lJointsVel(2) =  mSKinematicChain->getMaxVel(2);
		lJointsVel(3) =  mSKinematicChain->getMaxVel(3);
		lJointsVel(4) = -mSKinematicChain->getMaxVel(4);
		lJointsVel(5) =  mSKinematicChain->getMaxVel(5);
		lJointsVel(6) = -mSKinematicChain->getMaxVel(6);


		lJointsVel.Print();
		(mJacobianPos*lJointsVel).Print("final");

		lJoint += (lJointsVel *_dt);

		mSKinematicChain->setJoints(lJoint.Array());
		mThrowingJointTrj.SetRow(lJoint, 500+frame);

		if(frame==0){
			lJointThrowingStart = lJointsVel;
		}

		cout <<" "<< (mJacobianPos*lJointsVel).Norm() << endl;
	}

	lJointThrowingEnd = lJointsVel;
	mJointReady = mJointThrow - (lJointThrowingStart*_dt*70.0);
	mJointReady(4) = mJointThrow(4) - (lJointsVel(4)*_dt*0.0);
	mJointRest  = mThrowingJointTrj.GetRow(500+TARGET_CONT_FRAME-1) + (lJointThrowingEnd*_dt*0.0);

	for(int i=0;i<KUKA_DOF;i++){
		if( mJointReady(i) < mSKinematicChain->getMin(i) + 0.01 ){
			mJointReady(i) = mSKinematicChain->getMin(i) + 0.01;
		}else if( mJointReady(i) > mSKinematicChain->getMax(i) - 0.01 ){
			mJointReady(i) = mSKinematicChain->getMax(i) - 0.01;
		}

		if( mJointRest(i) < mSKinematicChain->getMin(i) + 0.01 ){
			mJointRest(i) = mSKinematicChain->getMin(i) + 0.01;
		}else if( mJointRest(i) > mSKinematicChain->getMax(i) - 0.01 ){
			mJointRest(i) = mSKinematicChain->getMax(i) - 0.01;
		}
	}

	mJointReady.Print();

	// first generate throwing motion
	lInitVel.Zero();
	lTargetVel = lJointThrowingStart;
	mPolyJointPlanner->SetConstraints(mJointReady, lInitVel, mJointThrow, lTargetVel, 1.0);
	for(int i=0; i<500; i++)
	{
		mPolyJointPlanner->Get((double)(i+1)*_dt, lJoint);
		mThrowingJointTrj.SetRow(lJoint, i);
	}

	// goto stop motion
	lInitJoint = mThrowingJointTrj.GetRow(500+TARGET_CONT_FRAME-1);
	lInitVel = lJointThrowingEnd;
	lTargetVel.Zero();
	mPolyJointPlanner->SetConstraints(lInitJoint, lInitVel, mJointRest, lTargetVel, 1.0);
	for(int i=0; i<1000; i++)
	{
		mPolyJointPlanner->Get((double)(i+1)*_dt, lJoint);
		mThrowingJointTrj.SetRow(lJoint, i+500+TARGET_CONT_FRAME);
	}
	mThrowingJointTrj.Save("/home/seungsu/trj.txt", 10, 500+TARGET_CONT_FRAME+1000);
}

extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    RandomThrowing* create(){return new RandomThrowing();}
    void destroy(RandomThrowing* module){delete module;}
}
