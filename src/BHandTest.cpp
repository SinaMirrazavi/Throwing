/*
 * BHandTest.cpp
 *
 *  Created on: Mar 19, 2013
 *      Author: seungsu
 */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "rosrt/rosrt.h"
#include "CDDynamics.h"
#include "ros/service.h"
#include "ros/service_server.h"

#include "BHandUtilities.h"
#include "BHandUtil/supervisory.h"

#define BHAND_STATE_TOPIC               "bhand_joint_state"
#define BHAND_CMD_TOPIC                 "bhand_joint_cmd"
#define SUPERVISORY_SERVICE_NAME 	"/bhand_controller/supervise"
#define BHAND_NUM_DOF 				4


int main(int argc, char **argv) {
	ros::init(argc, argv, "bhand_test");
	ros::NodeHandle nh;
	rosrt::init();

	MathLib::Vector lCurrentJoints(BHAND_NUM_DOF);

	ros::Rate r(100);

	rosrt::Subscriber<sensor_msgs::JointState> sub(3, nh ,BHAND_STATE_TOPIC );

//	ros::ServiceClient lServiceClient = nh.serviceClient(SUPERVISORY_SERVICE_NAME);
	 ros::ServiceClient lServiceClient = nh.serviceClient<BHandUtil::supervisory>(SUPERVISORY_SERVICE_NAME);
	 BHandUtil::supervisory srv;
	 srv.request.cmd = "rton";

	 if( lServiceClient.call(srv) )
	 {
	 }

	sensor_msgs::JointState template_msg;
	template_msg.position.resize(BHAND_NUM_DOF);
	template_msg.velocity.resize(BHAND_NUM_DOF);
	template_msg.effort.resize(BHAND_NUM_DOF);

	rosrt::Publisher<sensor_msgs::JointState> pub(nh.advertise<sensor_msgs::JointState>(BHAND_CMD_TOPIC, 1), 3, template_msg);
	sensor_msgs::JointStatePtr lJointControl = pub.allocate();

	// receive joint angle from it
	while(ros::ok())
	{
		ros::spinOnce();

		sensor_msgs::JointStateConstPtr lJointState = sub.poll();

		if(lJointState)
		{
			if(lJointState->position.size() != BHAND_NUM_DOF)
			{

			}
			else
			{
				for(int i=0; i<BHAND_NUM_DOF; i++)
				{
					lCurrentJoints(i) = lJointState->position[i];
				}
				lCurrentJoints.Print();

				sensor_msgs::JointStatePtr lJointControl = pub.allocate();


				lJointControl->position[0] = 0.1;
				lJointControl->position[1] = 0.1;
				lJointControl->position[2] = 1.1;
				lJointControl->position[3] = 0.1;

				pub.publish(lJointControl);

			}
		}
	}


}
