#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <sstream>

using namespace std;

/*
	Node for visualizing movements of robots in RViz.
	You should not edit this.
*/


struct State{
	double x,y;
	double ang;
};

State spawn1,spawn2;
vector<string> vehNames;
vector<tf::Transform> base_link_transform;
vector<geometry_msgs::PoseStamped> vehPose;
const double PI = 3.1415, SQRT2 = sqrt(2);
double Td = 0;
int timeStamp;

// ******************* INITIAL POSE CALLBACK ************************
void initialPoseReceivedCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	for(unsigned int i = 0; i < vehNames.size(); i++)
	{
		if(msg->header.frame_id.find(vehNames[i]) != string::npos)
		{
			base_link_transform[i].setOrigin( tf::Vector3(msg->pose.pose.position.x,  msg->pose.pose.position.y, msg->pose.pose.position.z) );

			tf::Quaternion q1;
			tf::quaternionMsgToTF(msg->pose.pose.orientation, q1);
			base_link_transform[i].setRotation( q1 );
		}
	}
}
// ******************************************************************

// ********************** CMD_VEL CALLBACK **************************
void cmdVelReceivedCallback(const geometry_msgs::Twist::ConstPtr &msg, const std::string &topic)
{
	for(unsigned int i = 0; i < vehNames.size(); i++)
	{
		if(topic.find(vehNames[i]) != string::npos)
		{
			tf::Quaternion q1;
			tf::quaternionMsgToTF(vehPose[i].pose.orientation, q1);
			double yaw = tf::getYaw(q1);

			double delta_yaw = msg->angular.z * Td;
			double new_yaw = yaw + delta_yaw;
			tf::Quaternion new_quat = tf::createQuaternionFromYaw( new_yaw );
			vehPose[i].pose.orientation = tf::createQuaternionMsgFromYaw( new_yaw );

			double delta_x =  (msg->linear.x * Td) * cos(new_yaw);
			double delta_y =  (msg->linear.x * Td) * sin(new_yaw);

			vehPose[i].pose.position.x += delta_x;
			vehPose[i].pose.position.y += delta_y;
			vehPose[i].pose.position.z = 0;

			base_link_transform[i].setOrigin( tf::Vector3(vehPose[i].pose.position.x,  vehPose[i].pose.position.y, vehPose[i].pose.position.z) );
			base_link_transform[i].setRotation( new_quat );
		}
	}
}
// ******************************************************************

double degreesToRadians(double deg){
	return deg * PI/180;
}
double radiansToDegrees(double deg){
	return deg * 180/PI;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "p3dxSim");
	ros::NodeHandle n;

  	vector<ros::Subscriber> subInitialPose;
  	vector<ros::Subscriber> subCmdVel;

  	vector<ros::Publisher> pubPose;
  	XmlRpc::XmlRpcValue vehicleNames;
		XmlRpc::XmlRpcValue vsa;
		XmlRpc::XmlRpcValue vsb;
  	// Getting vehicle names from the ROS Parameter Server
  	n.getParam("/vehicleNames", vehicleNames);
		n.getParam("/spawn1", vsa);
		n.getParam("/spawn2", vsb);

		n.getParam("/timeStamp", timeStamp);
		Td = 1./timeStamp;

  	ROS_ASSERT(vehicleNames.getType() == XmlRpc::XmlRpcValue::TypeArray);

		double A[3],B[3];
		for (int i = 0; i < 3;i++){
			A[i] = static_cast<double>(vsa[i]);
			B[i] = static_cast<double>(vsb[i]);												//casting from XML to CPP
		}
		spawn1.x = A[0];
		spawn1.y = A[1];
		spawn1.ang = A[2];
		spawn2.x = B[0];
		spawn2.y = B[1];
		spawn2.ang = B[2];

  	// Petlja po imenima svih vozila:
  	for (int32_t i = 0; i < vehicleNames.size(); ++i)
  	{
    	ROS_ASSERT(vehicleNames[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    	string vehicleName = static_cast<string>(vehicleNames[i]);


    	vehNames.push_back(vehicleName);

    	// Subscribers for receiving individual vehicle's initial pose
    	string topicName = "/" + vehicleName + "/initialpose";
    	subInitialPose.push_back(n.subscribe(topicName, 1, initialPoseReceivedCallback));

		// Subscribers for receiving individual vehicle's "cmd_vel"
    	topicName = "/" + vehicleName + "/cmd_vel_sim";
    	subCmdVel.push_back(n.subscribe<geometry_msgs::Twist>(topicName, 1, boost::bind(cmdVelReceivedCallback, _1, topicName)));

    	// Publishers for publishing simulated vehicle poses (used only for displaying vehicles within RViz by arrows)
    	topicName = "/" + vehicleName + "/simPose";
    	pubPose.push_back(n.advertise<geometry_msgs::PoseStamped>(topicName, 1));
  	}




	tf::Transform trans;
	vector<tf::Transform> transform(2);
	tf::Vector3 vect = tf::Vector3(spawn1.x, spawn1.y, 0.0), vect1 = tf::Vector3(0.0, 0.0, 0.0);
	tf::Vector3 zaxis = tf::Vector3(0.0,0.0,1.0);
	tf::Quaternion quat = tf::Quaternion(zaxis,degreesToRadians(spawn1.ang));
	transform[0].setRotation(quat);
	transform[0].setOrigin(vect);
	vect = tf::Vector3(spawn2.x, spawn2.y, 0.0);
	quat = tf::Quaternion(zaxis,degreesToRadians(spawn2.ang));
	transform[1].setRotation(quat);
	transform[1].setOrigin(vect);
	quat = tf::Quaternion(zaxis,degreesToRadians(0));
  trans.setRotation(quat);
	trans.setOrigin(vect1);
	for(unsigned int i = 0; i < vehNames.size(); i++)
	{
		geometry_msgs::PoseStamped pose;
		vehPose.push_back(pose);
		vehPose[i].header.frame_id = string("/"+vehNames[i]+"/map");

		base_link_transform.push_back(trans);
	}


	static tf::TransformBroadcaster br;
	ros::Rate loop_rate(timeStamp);

	while (ros::ok())
  	{

  		for(unsigned int i = 0; i < vehNames.size(); i++)
		{
			// transform broadcasting
  			br.sendTransform(tf::StampedTransform(base_link_transform[i], ros::Time::now(), string("/"+vehNames[i]+"/map"), string("/"+vehNames[i]+"/base_link")));
  			br.sendTransform(tf::StampedTransform(transform[i], ros::Time::now(), "/map", string("/"+vehNames[i]+"/map")));

			// publishing simulated vehicle poses (used only for the purpose of representing vehicles within RViz by arrows)
  			tf::pointTFToMsg(base_link_transform[i].getOrigin(), vehPose[i].pose.position);
			tf::quaternionTFToMsg(base_link_transform[i].getRotation(), vehPose[i].pose.orientation);
			vehPose[i].header.stamp = ros::Time::now();
  			pubPose[i].publish(vehPose[i]);
  		}

	  	ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
