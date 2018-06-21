#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <queue>
#include <vector>
#include <map>
#include <functional>

using namespace std;


/*
		Only task this short nodey has to do is transform current positions of the
		robots in state of configuration and calculate configuration length.
		Most of the functions and variables are already described in
		pathFollowing.cpp, so take a look there.
*/

const double PI = 3.1415;
struct State{
	//definition of state described in paper

	double x,y;
	double ang;
};

State currentPositionA,currentPositionB;
State spawn1,spawn2;
double resolution = 0;
int timeStamp = 0;

double degreesToRadians(double deg){
	return deg * PI/180;
}
double radiansToDegrees(double deg){
	return deg * 180/PI;
}

double standardizeAngleD(double angle){
	// gives angle in degrees in range <-180,180]
	if (angle <= -180)
		return angle += 360;
	if (angle > 180)
		return angle -= 360;
	return angle;
}

double standardizeAngleR(double angle){
	// gives angle in degrees in range <-180,180]
	if (angle <= -PI)
		return angle += 2*PI;
	if (angle > PI)
		return angle -= 2*PI;
	return angle;
}

void currentPositionCallbackA(const geometry_msgs::PoseStamped &pose){
	 geometry_msgs::Quaternion quat;
	 quat =  pose.pose.orientation;
	 double x = pose.pose.position.x;
	 double y = pose.pose.position.y;
	 double len = sqrt(x*x + y*y);
	 double ang = atan2(y,x) + degreesToRadians(spawn1.ang);
	 currentPositionA.x = (spawn1.x + len*cos(ang))/resolution;
	 currentPositionA.y = (spawn1.y + len*sin(ang))/resolution;
	 ang = round(2 * atan2(quat.z,quat.w) *180/PI) + spawn1.ang;
	 ang = standardizeAngleR(ang);
	 currentPositionA.ang = ang;
}


void currentPositionCallbackB(const geometry_msgs::PoseStamped &pose){
	 geometry_msgs::Quaternion quat;
	 quat =  pose.pose.orientation;
	 double x = pose.pose.position.x;
	 double y = pose.pose.position.y;
	 double len = sqrt(x*x + y*y);
	 double ang = atan2(y,x) + degreesToRadians(spawn2.ang);
	 currentPositionB.x = (spawn2.x + len*cos(ang))/resolution;
	 currentPositionB.y = (spawn2.y + len*sin(ang))/resolution;
	 ang = round(2 * atan2(quat.z,quat.w) *180/PI) + spawn2.ang;
	 ang = standardizeAngleR(ang);
	 currentPositionB.ang = ang;
}


	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
	{
		 resolution = msg->info.resolution;
	 }



int main(int argc, char **argv)
{
	ros::init(argc, argv, "load_maker");
	ros::NodeHandle lm;

	ros::Publisher cur_pos = lm.advertise<std_msgs::Int32MultiArray>("/currentPosition",10);
	ros::Subscriber m = lm.subscribe("/Alfa/map_loc",1,mapCallback);
	XmlRpc::XmlRpcValue vsa;
	XmlRpc::XmlRpcValue vsb;
	XmlRpc::XmlRpcValue names;
	double robotLength;
	lm.getParam("/spawn1", vsa);
	lm.getParam("/spawn2", vsb);
	lm.getParam("/vehicleNames",names);
	lm.getParam("/timeStamp", timeStamp);
	ros::Rate loop_rate(timeStamp);
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

	//calculateing configuration length (robotLength) and setting it as paramerer
	lm.setParam("/robotLength",sqrt( pow((B[0]-A[0]),2) + pow((B[1]-A[1]),2)));

	vector<string> vehicleNames;
	for (int32_t i = 0; i < names.size(); ++i)
	{
		ROS_ASSERT(names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		string vehicleName = static_cast<string>(names[i]);


		vehicleNames.push_back(vehicleName);
	}

	ros::Subscriber cpA = lm.subscribe("/" + vehicleNames[0] + "/simPose",1,currentPositionCallbackA);
	ros::Subscriber cpB = lm.subscribe("/" + vehicleNames[1] + "/simPose",1,currentPositionCallbackB);

	while (ros::ok() && resolution == 0){
		//waiting for resolution
			ros::spinOnce();
			loop_rate.sleep();
			continue;
	}

	while(ros::ok()){

	//making vector which contains informations about configuration
	/*
			state[0] = x coordinate of center of config;
			state[1] = y coordinate of center of config;
			state[2] = angle of config;
			state[3] = angle of the first robot;
			state[4] = angle of the second robot;

	*/
	std_msgs::Int32MultiArray state;
	state.data.clear();

	state.data.push_back(round((currentPositionA.x + currentPositionB.x) / 2));
	state.data.push_back(round((currentPositionA.y + currentPositionB.y) / 2));
	double deltaX = currentPositionB.x - currentPositionA.x;
	double deltaY = currentPositionB.y - currentPositionA.y;
	if (currentPositionA.ang == 180 && currentPositionB.ang == 180 && abs(deltaY) < 0.005){
		state.data.push_back(180);
	}
	else
	state.data.push_back(static_cast<int>(atan2(deltaY,deltaX)/PI * 180));
	state.data.push_back(currentPositionA.ang);
	state.data.push_back(currentPositionB.ang);
	cur_pos.publish(state);
	ros::spinOnce();
	loop_rate.sleep();

}


	return 0;

}
