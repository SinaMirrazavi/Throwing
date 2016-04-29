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

#include "FastThrowing.h"
#include "OptiTrack.h"

#define JOINT_STATE_TOPIC "/allegro/joint_state"
#define JOINT_CMD_TOPIC "/allegro/joint_cmd"

// optitrack
OptiTrack mTracking;
char mTrackingObjName[] = "obj_root";
char mBallName[] = "nestea_root";

//char mLocalIP[] = "128.178.145.250"; //lasapc15
//char mLocalIP[] = "128.178.145.88"; // lasapc34
char mLocalIP[] = "128.178.145.139"; //lasapc7

double **gOri;

CDDynamics *gFingerPlanner;
ros::Subscriber sub_handJoint;
ros::Publisher pub_handJoint;
double gFingerPos[16];
FILE *fid;

bool gIsPlannerINIT = false;


void initializeTracker()
{
	//local pc ip
	if(mTracking.Init(mLocalIP, true)<0){
		cout<<"ERROR: Cannot initialize"<<endl;
	}

	mTracking.enableWarnings(false);
	mTracking.loadCalibrationMatrix("/home/seungsu/devel/roscodes/BestCatchingPosture/data/model/BW_T.txt");

	vector<string> namelist = mTracking.GetRBodyNameList();

	if(namelist.size() == 0){
		cout<<"No rigid bodies found!!"<<endl;
	}
	else{
		for(unsigned int i=0;i<namelist.size();i++) cout<<i+1<<")  "<<namelist[i]<<endl;
	}

	//mTracking.enableRBody(mTrackingObjName, false);
	mTracking.enableRBody(mBallName, false);
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

FastThrowing::FastThrowing()
:RobotInterface(){
}
FastThrowing::~FastThrowing(){
}


void FastThrowing::initKinematics(void){
	mSKinematicChain = new sKinematics(KUKA_DOF, _dt);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	/*
	mSKinematicChain->setDH(0,  0.0,  0.310, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(132.0)*0.99);
	mSKinematicChain->setDH(1,  0.0,  0.000,-M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(132.0)*0.99);
	mSKinematicChain->setDH(2,  0.0,  0.400,-M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(128.0)*0.99);
	mSKinematicChain->setDH(3,  0.0,  0.000, M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(128.0)*0.99);
	mSKinematicChain->setDH(4,  0.0,  0.390, M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(204.0)*0.99);
	mSKinematicChain->setDH(5,  0.0,  0.000,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(184.0)*0.99); // reduced joint angle to save the fingers
	mSKinematicChain->setDH(6,  0.0,  0.117,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(184.0)*0.99);
	*/

	mSKinematicChain->setDH(0,   0.0,    0.310,       M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(132.0)*0.95);
	mSKinematicChain->setDH(1,   0.0,    0.000,      -M_PI_2, 0.0, 1,  DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(132.0)*0.95);
	mSKinematicChain->setDH(2,   0.0,    0.400,      -M_PI_2, 0.0, 1,  DEG2RAD(-160.), DEG2RAD(160.), DEG2RAD(128.0)*0.95);
	mSKinematicChain->setDH(3,   0.0,    0.000,       M_PI_2, 0.0, 1,  DEG2RAD(-100.), DEG2RAD(100.), DEG2RAD(128.0)*0.95);
	mSKinematicChain->setDH(4,   0.0,    0.390,       M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(204.0)*0.95);
	mSKinematicChain->setDH(5,   0.0,    0.000,      -M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(184.0)*0.95); // reduced joint angle to save the fingers
	//mSKinematicChain->setDH(6,  -0.05,  0.117+0.16,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(184.0)*0.95);
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

void FastThrowing::syncRobotChain(void)
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

RobotInterface::Status FastThrowing::RobotInit(){

	gOri = (double**) new double[3];
	for(int i=0;i<3;i++)
		gOri[i] = new double[3];

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

	initializeTracker();

	// target variable resize
	mJointReady.Resize(KUKA_DOF);
	mJointRest.Resize(KUKA_DOF);
	mJointThrow.Resize(KUKA_DOF);
	mThrowDir.Resize(3);

	mThrowingJointTrj.Resize(500+TARGET_CONT_FRAME+1000,KUKA_DOF);

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
	mOffsetYaw = DEG2RAD(8.0);
	mOffsetMag = 1.0;


	mCommand = COMMAND_NONE;
	mHandCommand = HANDCOMMAND_NONE;
	mStatus  = STATUS_NONE;

	AddConsoleCommand("test");
	AddConsoleCommand("job");
	AddConsoleCommand("home");
	AddConsoleCommand("throw");
	AddConsoleCommand("move");
	AddConsoleCommand("grasp");
	AddConsoleCommand("setym");
	AddConsoleCommand("getym");
	AddConsoleCommand("cal");


    return STATUS_OK;
}
RobotInterface::Status FastThrowing::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status FastThrowing::RobotStart(){

	nodeNetwork = mRobot->InitializeROS();
	mMsgJoint.position.resize(4*4);
	mMsgJoint.velocity.resize(4*4);
	sub_handJoint = nodeNetwork->subscribe(JOINT_STATE_TOPIC, 3, FingerJointCallback);
	pub_handJoint = nodeNetwork->advertise<sensor_msgs::JointState>(JOINT_CMD_TOPIC, 3);

    return STATUS_OK;
}    
RobotInterface::Status FastThrowing::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status FastThrowing::RobotUpdate(){
	static int lThrowingIndex=0;
	Vector ljoints(KUKA_DOF);
	Vector lBallPos(3), lPos(3);

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

    		mTracking.Update();
    		mTracking.getRBodyPosition(lBallPos.Array(), mBallName);
    		lBallPos.Print("ball");

    		mSKinematicChain->getEndPos(lPos.Array());
    		lPos.Print("end");

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
		default:
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
RobotInterface::Status FastThrowing::RobotUpdateCore(){
	Vector lPos(KUKA_FINGER_DOF);
	double lOri[3][3];

	static int lstaticFrame = 0;

	/*
	Vector lTargetPos(3);
	mTracking.Update();
	mTracking.getRBodyPosition(lTargetPos.Array(), mTrackingObjName );
	lTargetPos.Print();
*/

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
		gFingerPlanner->GetState(lPos);

		for(int i=0; i<16; i++){
			mMsgJoint.position[i] = lPos(i);
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
	for(int i=0; i<KUKA_DOF       ; i++) mJointAll(i         ) = mJointPos(i);
	for(int i=0; i<KUKA_FINGER_DOF; i++) mJointAll(i+KUKA_DOF) = 0.0;

	mActuatorsGroup.SetJointAngles(mJointPos);
	mActuatorsGroup.WriteActuators();
	mKinematicChain.Update();


	double lBallPos[3];
	double lEndPos[3];
	if( (mStatus ==STATUS_THROWJOINT_THROWING) && (lstaticFrame == 0 ) )
	{
    	fid = fopen("/home/seungsu/ball.txt", "w+");
    	lstaticFrame++;
	}
	else if(lstaticFrame > 0 )
	{
		mTracking.Update();
		mTracking.getRBodyPosition(lBallPos, mBallName);
		mTracking.getRBodyOrientation(lOri, mBallName);

		mSKinematicChain->setJoints(mJointPos.Array());
		mSKinematicChain->getEndPos(lEndPos);

		for(int i=0; i< 3; i++) fprintf(fid, "%lf ", lEndPos[i] );
		for(int i=0; i< 3; i++) fprintf(fid, "%lf ", lBallPos[i] );
		for(int i=0; i< 3; i++) fprintf(fid, "%lf ", lOri[i][0] );
		for(int i=0; i< 3; i++) fprintf(fid, "%lf ", lOri[i][1] );
		for(int i=0; i< 3; i++) fprintf(fid, "%lf ", lOri[i][2] );
		fprintf(fid, "\n");

		lstaticFrame++;
		if( lstaticFrame > 500*3)
		{
			fclose(fid);
			lstaticFrame = 0;
		}
	}


	ros::spinOnce();

	return STATUS_OK;
}
int FastThrowing::RespondToConsoleCommand(const string cmd, const vector<string> &args){
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
    else if(cmd=="throw"){
    	mHandCommand = HANDCOMMAND_GRASP;

    	mJointThrow.Set(cThrow, KUKA_DOF);

    	mThrowDir(0) = -1.0;
    	mThrowDir(1) =  0.0;
    	mThrowDir(2) =  1.0;
    	mThrowDir.Normalize();
    	mThrowMag = 10.0;


    	//mTracking.Update();
    	//mTracking.getRBodyPosition(lTargetPos.Array(), mTrackingObjName );
    	lTargetPos(0) = -3.9;
    	lTargetPos(1) =  0.0;
    	lTargetPos(2) =  0.0;

    	/*
    	lTargetPos(0) = -3.4;
    	lTargetPos(1) = -0.957755;
    	lTargetPos(2) =  0.0;
    	*/


    	if( lTargetPos.Norm() < 0.0001 )
    	{
    		cout << "please check vision system " << endl;
    	}
    	else{

			//lTargetPos(2) -=  (0.07+0.05);
			lTargetPos.Print();

			//ros::Time tstart = ros::Time::now();

			calculateThrowJointTrajectory(lTargetPos);
			//_calculateThrowJointTrajectory();

			//ros::Time tend = ros::Time::now();
			//printf("%lf \n", 1e-6*(tend - tstart).nsec );


			mCommand = COMMAND_THROWJOINT_READY;
    	}
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



void FastThrowing::calculateThrowJointTrajectory(Vector3& targetPos)
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

	lVel.Print("velocity");
	lPos.Print("position");
	//printf("%lf \n",  RAD2DEG(gamma) );


	// find magnitude
	mSKinematicChain->setJoints(mJointThrow.Array());
	mSKinematicChain->getJacobianPos(mJacobianPos);
	mSKinematicChain->getEndPos(lPos.Array());
	lVel = mJacobianPos*lJointsVel;

	double lDuration;
	double lKp = 1.0;
	lError.Zero();

	for(int i=0; i<50; i++){
		lKp  *= (1.0 -(lError(0)*0.01));
		lKp  *= (1.0 +(lError(2)*0.01));

		mBallistic->SetPosVel(lPos.Array(), (lVel*lKp).Array());
		lDuration = mBallistic->CalSmallestError(targetPos, lError);

		if( lError.Norm() < 0.001) break;
		cout << "error : " << lError(0) << " " << lError(1) << " " << lError(2) << ", kp " <<  lKp << endl;
	}
cout<<"mOffsetMag "<<mOffsetMag<<endl;
	lKp *= mOffsetMag;
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

	mThrowingJointTrj.Save("/home/seungsu/trj.txt", 10, 500+TARGET_CONT_FRAME+1000);

cout<<"END OF THE CODE"<<endl;
}


void FastThrowing::_calculateThrowJointTrajectory(void)
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

		/*
		// Set maximum joint velocity
		for(int i=0;i<KUKA_DOF;i++){
			mJointVelLimitsDn(i) = -mSKinematicChain->getMaxVel(i);
			mJointVelLimitsUp(i) =  mSKinematicChain->getMaxVel(i);

			if( (lJoint(i)-mSKinematicChain->getMin(i) )< mJointVelocityLimitsDT(i)){
				mJointVelLimitsDn(i) *= (lJoint(i)-mSKinematicChain->getMin(i))/mJointVelocityLimitsDT(i);
			}else if( (mSKinematicChain->getMax(i)-lJoint(i))<mJointVelocityLimitsDT(i)){
				mJointVelLimitsUp(i) *= (mSKinematicChain->getMax(i)-lJoint(i))/mJointVelocityLimitsDT(i);
			}
		}
		mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);
		*/

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
		/*
		// to avoid go to near joint limits
		for(int i=0;i<KUKA_DOF;i++){
			mJointWeights(i) = 1.0;
			if( ( lJoint(i)< mSKinematicChain->getMin(i)+DEG2RAD(5.0) ) && (lJointsVel(i) < 0.0)){
				mJointWeights(i) = (lJoint(i) - mSKinematicChain->getMin(i)) / DEG2RAD(5.0);
				if( mJointWeights(i) < 0 ) mJointWeights(i) = 0.0;

				lJointsVel(i) *= mJointWeights(i);
				cout << i << ": up " << endl;
			}
			else if( (lJoint(i)> mSKinematicChain->getMax(i)-DEG2RAD(5.0)) && (lJointsVel(i) > 0.0)){
				mJointWeights(i) = (mSKinematicChain->getMax(i)-lJoint(i)) / DEG2RAD(5.0);
				if( mJointWeights(i) < 0 ) mJointWeights(i) = 0.0;

				lJointsVel(i) *= mJointWeights(i);
				cout << i << ": dn " << endl;
			}
		}
		*/

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
    FastThrowing* create(){return new FastThrowing();}
    void destroy(FastThrowing* module){delete module;}
}

