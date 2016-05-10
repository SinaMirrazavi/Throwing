/*
 * SkeletonTest.cpp
 *
 *  Created on: Jan 8, 2013
 *      Author: seungsu
 */

#include "MathLib.h"
#include "OptiTrack.h"
#include "ros/ros.h"

enum ENUM_COMMAND
{
	COMMAND_START_RECORDING,
	COMMAND_STOP_RECORDING,
	COMMAND_QUIT,
	COMMAND_NONE
};

OptiTrack mTracking;
char mTrackingObjName[] = "Skeleton4";
int mSkeletonSid;
//char mLocalIP[] = "128.178.145.88"; // lasapc34
char mLocalIP[] = "128.178.145.139"; //lasapc7

int mIndex = 0;
FILE *fid_skeleton;
FILE *fid_marker;

ENUM_COMMAND mCommand = COMMAND_NONE;

void initializeTracker()
{
	//local pc ip
	if(mTracking.Init(mLocalIP, false)<0){
		cout<<"ERROR: Cannot initialize"<<endl;
	}

	mTracking.enableWarnings(false);
	mTracking.loadCalibrationMatrix("/home/seungsu/devel/roscodes/BestCatchingPosture/data/model/BW_T.txt");

	cout<<"Getting name list"<<endl;
	//vector<string> namelist = mTracking.GetRBodyNameList();
	//vector<string> sktlist = mTracking.GetSkeletonNameList();

	//if(namelist.size() == 0){
	//	cout<<"No rigid bodies found!!"<<endl;
	//}
	//else{
	//	for(unsigned int i=0;i<namelist.size();i++) cout<<i+1<<")  "<<namelist[i]<<endl;
	//}

	//if(sktlist.size() == 0){
	//	cout<<"No rigid bodies found!!"<<endl;
	//}
	//else{
	//	for(unsigned int i=0;i<sktlist.size();i++) cout<<i+1<<")  "<<sktlist[i]<<endl;
	//}

	cout<<"Initializing finished"<<endl;
	mSkeletonSid = mTracking.enableSkeleton(mTrackingObjName, true);
	mTracking.enableRBody("ball_root", false);

}

void WorkingThread(void)
{
	int lFrameNumber;
	double lPos[3];
	SPosture lBody[24];
	Skeleton sBody;
	initializeTracker();

	ros::Rate loop_rate(240);

	while(true)
	{
		if( mCommand == COMMAND_START_RECORDING )
		{
			mTracking.Update();

			// save skeleton data
			mTracking.getSkeleton(sBody, mSkeletonSid);
			mTracking.getSkeleton(lBody, mSkeletonSid);

			mTracking.getRBodyPosition(lPos, "ball_root");
			lFrameNumber = mTracking.getFrameNumber();

			for(int i=0; i<sBody.nRigidBodies; i++)
			{
				fprintf(fid_skeleton, "%lf %lf %lf %lf %lf %lf %lf %lf %lf ",
						lBody[i].pos[0], lBody[i].pos[1], lBody[i].pos[2],
						lBody[i].orient[0][0], lBody[i].orient[1][0], lBody[i].orient[2][0],
						lBody[i].orient[0][1], lBody[i].orient[1][1], lBody[i].orient[2][1] );

				for(int j=1; j<sBody.RBodies[i].nRigidMarkers; j++)
				{
					fprintf(fid_marker, "%lf %lf %lf ", sBody.RBodies[i].RMarkers[j].x, sBody.RBodies[i].RMarkers[j].y, sBody.RBodies[i].RMarkers[j].z );
				}
			}

			fprintf(fid_marker, "%lf %lf %lf %d \n", lPos[0], lPos[1], lPos[2], lFrameNumber);
			fprintf(fid_skeleton, "\n");
		}

		if(mCommand == COMMAND_STOP_RECORDING)
		{
			fclose(fid_skeleton);
			fclose(fid_marker);

			mCommand == COMMAND_NONE;
		}

		loop_rate.sleep();
	}
}

void DispCommand()
{
	cout << "Command List" << endl;
	cout << "	start : start recording " << endl;
	cout << "	stop  : stop  recording " << endl;
	cout << "---------------------------" << endl;
	cout << ">> ";
}

int main(int argc, char** argv)
{
	char fname[1024];
	char cmd[1024];

	ros::Time::init();

	boost::thread *working_thread;
	working_thread = new boost::thread(&WorkingThread);

	sleep(2);
	while(mCommand != COMMAND_QUIT){

		DispCommand();

		gets(cmd);

		if(strcmp(cmd, "start") ==0 )
		{
			sprintf(fname, "./data/skeleton_%02d.txt", mIndex);
			fid_skeleton = fopen(fname, "w+");

			sprintf(fname, "./data/marker_%02d.txt", mIndex);
			fid_marker  = fopen(fname, "w+");
			mIndex++;

			mCommand = COMMAND_START_RECORDING;
		}
		else if(strcmp(cmd, "stop") ==0 )
		{
			if(mCommand== COMMAND_START_RECORDING)
			{
				mCommand == COMMAND_STOP_RECORDING;
			}
		}
		else if(strcmp(cmd, "quit") ==0 )
		{
			mCommand = COMMAND_QUIT;
		}
	}

	return -1;
}
