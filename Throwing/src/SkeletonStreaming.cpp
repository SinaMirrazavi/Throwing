/*
 * SkeletonStreaming.cpp
 *
 *  Created on: Apr 19, 2013
 *      Author: seungsu
 */



#include "MathLib.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"
#include "OptiTrack.h"
#include "ros/ros.h"
#include "SkeletonStreaming.h"

// tracking
OptiTrack mTracking;

// ros topic
ros::Publisher rigidbody_pub;
ros::Subscriber rigidbody_sub;
geometry_msgs::PoseStamped msg;

char mBallName[] = "ball_root";
char mObjName[] = "nestea_root";
//char mSkeletonName[] = "SkeletonSunhye";
char mSkeletonName[] = "SkeletonKlas";
int mSkeletonSid;

char mLocalIP[] = "128.178.145.250"; //lasapc15
//char mLocalIP[] = "128.178.145.88"; // lasapc34
//char mLocalIP[] = "128.178.145.139"; //lasapc7
Skeleton mSkeleton;
double mObjPos[3];

MathLib::Vector3 mShoulderPos;
double  mArmLength;


ENUM_VISIONSTATUS mStatus = VISIONSTATUS_NONE;
ENUM_VISIONCOMMAND mCommand = VISIONCOMMAND_NONE;


void ThrowingMsgCallback(const std_msgs::Int16& rsvmsg)
{
	switch( rsvmsg.data )
	{
		case VISIONCOMMAND_SKELETONINFO :
	 		msg.pose.position.x = mShoulderPos(0);
	 		msg.pose.position.y = mShoulderPos(1);
	 		msg.pose.position.z = mShoulderPos(2);

	 		msg.pose.orientation.w = mArmLength;
	 		rigidbody_pub.publish(msg);
	 		printf("%lf %lf %lf, length %lf \n", mShoulderPos(0), mShoulderPos(1), mShoulderPos(2), mArmLength);
			break;
		case VISIONCOMMAND_STARTRECORDING :

			mCommand = VISIONCOMMAND_STARTRECORDING;
			break;
		case VISIONCOMMAND_STOPRECORDING :

			mCommand = VISIONCOMMAND_STOPRECORDING;
			break;
		default:
			mCommand = VISIONCOMMAND_NONE;
			break;
	}

}


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
	mSkeletonSid = mTracking.enableSkeleton(mSkeletonName, true);
	mTracking.enableRBody(mBallName, false);
	mTracking.enableRBody(mObjName, false);
}


int main(int argc, char** argv)
{
    // initialize publishing
    ros::init(argc, argv, "skeleton");
    ros::NodeHandle n;
    rigidbody_pub = n.advertise<geometry_msgs::PoseStamped>(THROWING_STATE_TOPIC, 3);
	rigidbody_sub = n.subscribe(THROWING_CMD_TOPIC, 3, ThrowingMsgCallback);

    ros::Time stamp;

    double lBallPos[3];
    MathLib::Vector lPos(3);

    initializeTracker();

    FILE *fid;
    char fname[100];
    int mIndex = 0;
    int mFrame = 0;
    ros::Rate lRate(250); // 250 hz

    MathLib::Matrix lTrj(250*15, 600);
    SPosture pskeleton[24];

    while(ros::ok())
	{
    	if( mSkeletonSid < 0) break;

    	mTracking.Update();
    	stamp = ros::Time::now();

		// record skeleton data and ball position
		mTracking.getSkeleton(pskeleton, mSkeletonSid);
		mTracking.getSkeleton(mSkeleton, mSkeletonSid);
		mTracking.getRBodyPosition(lBallPos, mBallName);
//		mTracking.getRBodyPosition(mObjPos, mObjName);

//		for(int i=0; i<24; i++ )
//			cout << mSkeleton.RBodies[i].nRigidMarkers << " ";
//		cout << endl;
//
//		printf("%lf %lf %lf, %lf %lf %lf, %lf %lf %lf\n",
//				 pskeleton[12].markerpos[0][0],
//				 pskeleton[12].markerpos[0][1],
//				 pskeleton[12].markerpos[0][2],
//				 pskeleton[12].markerpos[1][0],
//				 pskeleton[12].markerpos[1][1],
//				 pskeleton[12].markerpos[1][2],
//				 pskeleton[12].markerpos[2][0],
//				 pskeleton[12].markerpos[2][1],
//				 pskeleton[12].markerpos[2][2]);

//				printf("%lf %lf %lf, %lf %lf %lf, %lf %lf %lf\n",
//						pskeleton[0].pos[0],
//						pskeleton[0].pos[1],
//						pskeleton[0].pos[2],
//						pskeleton[1].pos[0],
//						pskeleton[1].pos[1],
//						pskeleton[1].pos[2],
//						pskeleton[2].pos[0],
//						pskeleton[2].pos[1],
//						pskeleton[2].pos[2]);

				mShoulderPos(0) =  pskeleton[12].markerpos[2][0];
				mShoulderPos(1) =  pskeleton[12].markerpos[2][1];
				mShoulderPos(2) =  pskeleton[12].markerpos[2][2];

				lPos(0) = pskeleton[14].markerpos[0][0];
				lPos(1) = pskeleton[14].markerpos[0][1];
				lPos(2) = pskeleton[14].markerpos[0][2];

//				mShoulderPos(0) =  pskeleton[7].markerpos[2][0];
//				mShoulderPos(1) =  pskeleton[7].markerpos[2][1];
//				mShoulderPos(2) =  pskeleton[7].markerpos[2][2];
//
//				lPos(0) = pskeleton[9].markerpos[0][0];
//				lPos(1) = pskeleton[9].markerpos[0][1];
//				lPos(2) = pskeleton[9].markerpos[0][2];

		mArmLength = (lPos-mShoulderPos).Norm();

//		mShoulderPos(0) = mObjPos[0];
//		mShoulderPos(1) = mObjPos[1];
//		mShoulderPos(2) = mObjPos[2];

    	switch( mCommand )
		{
			case VISIONCOMMAND_STARTRECORDING :
				// create file
				lTrj.Zero();
				mFrame = 0;
				cout << "start recording" << endl;
				//fid = fopen(fname, "w+");

				mStatus = VISIONSTATUS_RECORDING;
				break;
			case VISIONCOMMAND_STOPRECORDING :
				//fclose(fid);
				sprintf(fname, "./data/humancapture_%02d.txt", mIndex);
				lTrj.Save(fname, 8, mFrame);
				cout << fname << " : saved" << endl;
				mIndex++;

				break;
			default :
				break;
		}
    	mCommand = VISIONCOMMAND_NONE;

    	if( mStatus == VISIONSTATUS_RECORDING )
    	{
			// save it into a file

    		//    		fprintf(fid, "%lf %lf %lf ", lBallPos[0], lBallPos[1], lBallPos[2] );
    		//			fprintf(fid, "%d %d ", mTracking.getFrameNumber(), mSkeleton.nRigidBodies);
    		//			for(int i=0; i<mSkeleton.nRigidBodies; i++)
    		//			{
    		//				fprintf(fid, "%lf %lf %lf %lf %lf %lf %lf ",
    		//						mSkeleton.x, mSkeleton.y, mSkeleton.z,
    		//						mSkeleton.qx, mSkeleton.qy, mSkeleton.qz, mSkeleton.qw );
    		//
    		//				fprintf(fid, "%d ", mSkeleton.RBodies[i].nRigidMarkers);
    		//				for(int j=1; j<mSkeleton.RBodies[i].nRigidMarkers; j++)
    		//				{
    		//					fprintf(fid, "%lf %lf %lf ",
    		//							mSkeleton.RBodies[i].RMarkers[j].x,
    		//							mSkeleton.RBodies[i].RMarkers[j].y,
    		//							mSkeleton.RBodies[i].RMarkers[j].z );
    		//				}
    		//			}
    		//			fprintf(fid, "\n");

    		int lIndex=0;
    		lTrj(mFrame, lIndex++) = lBallPos[0];
    		lTrj(mFrame, lIndex++) = lBallPos[1];
    		lTrj(mFrame, lIndex++) = lBallPos[2];
    		lTrj(mFrame, lIndex++) = mTracking.getFrameNumber();
    		lTrj(mFrame, lIndex++) = mSkeleton.nRigidBodies;

			for(int i=0; i<mSkeleton.nRigidBodies; i++)
			{
				lTrj(mFrame, lIndex++) = mSkeleton.RBodies[i].x;
				lTrj(mFrame, lIndex++) = mSkeleton.RBodies[i].y;
				lTrj(mFrame, lIndex++) = mSkeleton.RBodies[i].z;
				lTrj(mFrame, lIndex++) = mSkeleton.RBodies[i].qx;
				lTrj(mFrame, lIndex++) = mSkeleton.RBodies[i].qy;
				lTrj(mFrame, lIndex++) = mSkeleton.RBodies[i].qz;
				lTrj(mFrame, lIndex++) = mSkeleton.RBodies[i].qw;
				lTrj(mFrame, lIndex++) = mSkeleton.RBodies[i].nRigidMarkers;

				for(int j=0; j<mSkeleton.RBodies[i].nRigidMarkers; j++)
				{
					lTrj(mFrame, lIndex++) = pskeleton[i].markerpos[j][0];
					lTrj(mFrame, lIndex++) = pskeleton[i].markerpos[j][1];
					lTrj(mFrame, lIndex++) = pskeleton[i].markerpos[j][2];
				}
			}

			mFrame++;
    	}

    	lRate.sleep();
    	ros::spinOnce();
	}
}
