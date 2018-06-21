#include <sys/time.h>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
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
		This node is for path following. It has to subscribe to current and start positions
		of robots, current state of configuration and path itself so it could calculateNextAngle
		commands for robot control.

*/




struct State{
		//definition of state described in paper
	double x,y;
	double ang;
};

const double PI = 3.1415, SQRT2 = sqrt(2);

double lookaheadDistance, linearVelocity,angularVelocity, positionTolerance = 0.02,robotLength = 0, delta = 0; //parameters from launch file
int timeStamp;

bool done = false;
bool pathPruned = false;
bool pathSet = false;
double resolution = 0;	// map resolution because path is if form of cells (not in meters!)
int pathLength = 0;
int first = 0;			//which robot is first at the beginning(can be 1 or 2)
nav_msgs::Path globalPath,transformedPath;  //path in map coordinate system and in robot's
State currentPositionC; //current state of configuration
State currentPosition1,currentPosition2,spawn1,spawn2; //current and start states of robots
State currentPosition1Real,currentPosition2Real;   //current states of robots from encoders (real robots)
double previousAngle=0,currentAngle=0,deltaAngle=0,deltaAngleR=0, nextAngle;  //angles (will be described in code)
int inFront = 0,front = 0;  // which robot is in front (0-None   1-First  2-Second)
int done1E = 0,done2E = 0,done1S=0,done2S=0; //will be described
int phase1 = 1,phase2 = 0,phase3 = 0,start = 0; //will be described
int isOnlySimulation = 1;



void initializePurePursuit()
{
    done = false;
    pathPruned = false;
}

double degreesToRadians(double deg){
	return deg * PI/180;
}
double radiansToDegrees(double deg){
	return round(deg * 180/PI);
}

double otherAngleD(double ang){
	//finds other possible angle of confuguration in degrees
	if (ang > 0) return ang-180;
	if (ang < 0) return ang+180;
	return ang;
}

double otherAngleR(double ang){
	//finds other possible angle of confuguration in radians
	if (ang > 0) return ang-PI;
	if (ang < 0) return ang+PI;
	return ang;
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


double standardizeConfigD(double angle){
	// gives angle in degrees in range <-90,90]
	if (angle <= -90)
		return angle += 180;
	if (angle > 90)
		return angle -= 180;
	return angle;
}

double euclideanDistance(State position,State goal){
	double ydist,xdist;
	ydist = (goal.y - position.y);
	xdist = (goal.x - position.x);
	return sqrt(pow(ydist,2) + pow(xdist,2));
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	 resolution = msg->info.resolution;
}

void currentPositionCallback(const std_msgs::Int32MultiArray &pose){
	//current state of configuration in standardised units
	currentPositionC.x = pose.data[0] * resolution;
	currentPositionC.y = pose.data[1] * resolution;
	currentPositionC.ang = degreesToRadians(pose.data[2]);
 }


 /*
 		As described in paper, coordinate transformations are needed for expresing current positions
		in global coordinate system
 */

 void currentPositionCallback1(const geometry_msgs::PoseStamped &pose){
 	geometry_msgs::Quaternion quat;
 	quat =  pose.pose.orientation;
	double x = pose.pose.position.x;
	double y = pose.pose.position.y;
	double len = sqrt(x*x + y*y);
	double ang = atan2(y,x) + degreesToRadians(spawn1.ang);
 	currentPosition1.x = (spawn1.x + len*cos(ang));
 	currentPosition1.y = (spawn1.y + len*sin(ang));
	ang = 2 * atan2(quat.z,quat.w) + degreesToRadians(spawn1.ang);
	ang = standardizeAngleR(ang);
 	currentPosition1.ang = ang;
 }

 void currentPositionCallback2(const geometry_msgs::PoseStamped &pose){
 	geometry_msgs::Quaternion quat;
 	quat =  pose.pose.orientation;
	double x = pose.pose.position.x;
	double y = pose.pose.position.y;
	double len = sqrt(x*x + y*y);
	double ang = atan2(y,x) + degreesToRadians(spawn2.ang);
 	currentPosition2.x = (spawn2.x + len*cos(ang));
 	currentPosition2.y = (spawn2.y + len*sin(ang));
	ang = 2 * atan2(quat.z,quat.w) + degreesToRadians(spawn2.ang);
	ang = standardizeAngleR(ang);
 	currentPosition2.ang = ang;
 }

 void currentPositionCallback1Real(const nav_msgs::Odometry &msg){
 	geometry_msgs::Quaternion quat;
 	quat =  msg.pose.pose.orientation;
	double x = msg.pose.pose.position.x;
	double y = msg.pose.pose.position.y;
	double len = sqrt(x*x + y*y);
	double ang = atan2(y,x) + degreesToRadians(spawn1.ang);
 	currentPosition1Real.x = (spawn1.x + len*cos(ang));
 	currentPosition1Real.y = (spawn1.y + len*sin(ang));
	ang = 2 * atan2(quat.z,quat.w) + degreesToRadians(spawn1.ang);
	ang = standardizeAngleR(ang);
	isOnlySimulation = 0;  //if this callback function is called, program is
												 //receiving informations from robots
 }

 void currentPositionCallback2Real(const nav_msgs::Odometry &msg){
 	geometry_msgs::Quaternion quat;
 	quat =  msg.pose.pose.orientation;
	double x = msg.pose.pose.position.x;
	double y = msg.pose.pose.position.y;
	double len = sqrt(x*x + y*y);
	double ang = atan2(y,x) + degreesToRadians(spawn2.ang);
 	currentPosition2Real.x = (spawn2.x + len*cos(ang));
 	currentPosition2Real.y = (spawn2.y + len*sin(ang));
	ang = 2 * atan2(quat.z,quat.w) + degreesToRadians(spawn2.ang);
	ang = standardizeAngleR(ang);
 	currentPosition2Real.ang = ang;
 }



void globalPathCallback(const nav_msgs::Path& global){

	geometry_msgs::PoseStamped pos;

	if (!pathSet){
		//first time path is rewriten from PoseStamped to State
		pathSet = true;
		pathLength = global.poses.size();

		for(int i = 0; i < global.poses.size();i++){
	   	pos = global.poses[i];
    	globalPath.poses.push_back(pos);
			double x = global.poses[i].pose.position.x - currentPositionC.x;
			double y = global.poses[i].pose.position.y - currentPositionC.y;
			pos.pose.position.x = x;
			pos.pose.position.y = y;
			transformedPath.poses.push_back(pos);
  	}

		/*
			A value of variable first is found here. Algorithm calculates distances of the robots with first
			state on the path, and shorter distance means that robot is in front. If diffenrece is really
			small, then the robot with the shorter distance to the end is first.
		*/
		State state;
		state.x = global.poses[1].pose.position.x;
		state.y = global.poses[1].pose.position.y;
		double A = euclideanDistance(currentPosition1,state);
		double B = euclideanDistance(currentPosition2,state);
		if (A < B) first = 1;
		else if (B < A) first = 2;
		if (abs(A - B)<positionTolerance){
			state.x = (global.poses.end()--)->pose.position.x;
			state.y = (global.poses.end()--)->pose.position.y;
			double A = euclideanDistance(currentPosition1,state);
			double B = euclideanDistance(currentPosition2,state);
			if (A < B) first = 1;
			else first = 2;
		}

	}
  else{
		//updates transformedPath
		for (int i = 0; i < globalPath.poses.size();i++){
			pos = globalPath.poses[i];
			double x = globalPath.poses[i].pose.position.x - currentPositionC.x;
			double y = globalPath.poses[i].pose.position.y - currentPositionC.y;
			pos.pose.position.x = x;
			pos.pose.position.y = y;
			transformedPath.poses[i] = pos;

		}
	}

}

vector<State> findRobotPositions(double currentAngle,double x,double y){
  /*
		Given the angle of next configuration state and positions,
		this function calculates where robots should be. With
		constant distance between the robots, those positions
		can be hardcoded, so thats what I did.
	*/

	vector<State> dest;
	State state;
	state.ang = round(currentAngle * 180/PI);
	double l;
	if(state.ang == 0 || state.ang == -180 || state.ang == 180){
		state.y = y;
		state.x = x + robotLength/2;
		dest.push_back(state);
		state.x = x - robotLength/2;
		dest.push_back(state);
	}
	else if (state.ang == 90 || state.ang == -90){
		state.x = x;
		state.y = y + robotLength/2;
		dest.push_back(state);
		state.y = y - robotLength/2;
		dest.push_back(state);
	}
	else if (state.ang == 45 || state.ang == -135){
		l = SQRT2/4 * robotLength;
		state.x = x + l;
		state.y = y + l;
		dest.push_back(state);
		state.x = x - l;
		state.y = y - l;
		dest.push_back(state);
	}
	else if (state.ang == -45 || state.ang == 135){
		l = SQRT2/4 * robotLength;
		state.x = x + l;
		state.y = y - l;
		dest.push_back(state);
		state.x = x - l;
		state.y = y + l;
		dest.push_back(state);
	}
	return dest;
}


// Find the nearest point on the path
void prunePath()
{
    double diffX,diffY;
    double pointDistanceSq = 0;
    int index = 0;

		//finding state on the path closest to the config center
    diffX = transformedPath.poses[0].pose.position.x;
    diffY = transformedPath.poses[0].pose.position.y;
    double previousDistanceSq = diffX * diffX + diffY * diffY;

    if(transformedPath.poses.size() > 0)
    {
        for(unsigned int i = 1; i < transformedPath.poses.size(); i++)
        {
            diffX = transformedPath.poses[i].pose.position.x;
            diffY = transformedPath.poses[i].pose.position.y;
            pointDistanceSq = diffX * diffX + diffY * diffY;

            if (pointDistanceSq <= previousDistanceSq)
            {
                previousDistanceSq = pointDistanceSq;
                index = i;
            }
        }
        diffX = transformedPath.poses[index].pose.position.x;
        diffY = transformedPath.poses[index].pose.position.y;

        //erasing all positions on the path before the one closest to the config center
				if(transformedPath.poses.size() > 1)
				  for(int i = 0; i < index; i++)
	        {
	            transformedPath.poses.erase(transformedPath.poses.begin());
	            globalPath.poses.erase(globalPath.poses.begin());
	        }
    }
    pathPruned = true;
}

double findAngularVelocity(double angle, double currentAng){
		//in degrees
		double delta = standardizeAngleD(angle - currentAng);
	  if (delta > 0 && delta <= 181)
			return angularVelocity;
		if (delta > 0)
		 return -angularVelocity;
	  if (delta < 0 && delta > -181)
		 return -angularVelocity;
 		if (delta < 0)
 		 return angularVelocity;


}

void calculateVelocities(geometry_msgs::Twist& commandVelocity1,State point1,geometry_msgs::Twist& commandVelocity2,State point2,
												 geometry_msgs::Twist& commandVelocity1R,geometry_msgs::Twist& commandVelocity2R){

	double angularVelocity1,angularVelocity2;

	if (previousAngle == currentAngle){
		//if there is no change in angle on the path, speeds are calculated
		//as it was derived in the paper

			double curvature1 = (2* point1.y)/(point1.x * point1.x + point1.y * point1.y);
			double curvature2 = (2* point2.y)/(point2.x * point2.x + point2.y * point2.y);

			commandVelocity1.linear.x  = linearVelocity;
			commandVelocity2.linear.x  = linearVelocity;
			commandVelocity1R.linear.x  = linearVelocity;
			commandVelocity2R.linear.x  = linearVelocity;

			angularVelocity1 = linearVelocity * curvature1;
			angularVelocity2 = linearVelocity * curvature2;

			commandVelocity1.angular.z = angularVelocity1;
			commandVelocity2.angular.z = angularVelocity2;
			commandVelocity1R.angular.z = angularVelocity1;
			commandVelocity2R.angular.z = angularVelocity2;

				deltaAngleR = 0;
	}

	else{  //when change in angle occurs

		/*
			Main program is constantly calling this function to calculate speeds.
			Because of that, process of changing the config angle is done in phases.
			By default, phase1 is set. When phase1 is done, it triggers phase2 and
			shuts itself down, phase2 then triggers phase3 etc.

			In phase1 robots position themselves perpendicular to current state of config.
			deltaAngle is the variable which tells which way should robots be rotated.
			They rotate in place.

			In phase2 robots rotate around center of config in next angle of config

			In phase3 robots rotate in place to the angle which is following path.

			A lot of code will be copy-pasted, because speeds in each phase are
			calculated in the same way.

		  Variables doneXS and doneXE are triggered when robot X is done with
			currently active phase. S - simulation, E - experiment
		*/



		double angle1,angle2; //angles of front and back robots
		deltaAngleR = currentAngle - previousAngle;
		/*
			It's important to know the diffenrece between deltaAngleR and deltaAngle.
			deltaAngleR does not have to be standardised, while deltaAngle is standardised.
			That's because algorithm wants to find smalest angle for which it has to rotate
			and it is done in deltaAngle. deltaAngleR is here just so currentAngle can be
			calculated as currentAngle = previousAngle + deltaAngleR.
		*/


		angle1 = currentAngle;
		angle2 = previousAngle;

		//in range < 0 , 2PI ]

		deltaAngle = standardizeConfigD(radiansToDegrees(angle1 - angle2));

		if(phase1){

			if (deltaAngle >= 0){
				angle1 = round(radiansToDegrees(previousAngle) + 90);
				angle2 = round(radiansToDegrees(previousAngle) - 90);
			}
			else{
				angle1 = round(radiansToDegrees(previousAngle) - 90);
				angle2 = round(radiansToDegrees(previousAngle) + 90);
			}
			angle1 = standardizeAngleD(angle1);
			angle2 = standardizeAngleD(angle2);
			commandVelocity1.linear.x  = 0;
			commandVelocity2.linear.x  = 0;
			commandVelocity1R.linear.x  = 0;
			commandVelocity2R.linear.x  = 0;
			static int up,right;
			double static startAngle1,startAngle2;
			if (!start){
				start = 1;
				startAngle1 = radiansToDegrees(currentPosition1.ang);
				startAngle2 = radiansToDegrees(currentPosition2.ang);
				double x = currentPosition1.x - currentPositionC.x;
				double y = currentPosition1.y - currentPositionC.y;

				if (y > positionTolerance)
					up = 1;
				else if (abs(y) <positionTolerance)
					up = 0;
				else up = 2;

				if (x > positionTolerance)
					right = 1;
				else if (abs(x) <positionTolerance)
					right = 0;
				else right = 2;
			}
			//front going counterclockwise
			//back going clockwise
			if (up == 1 || (up == 0 && right == 1)){  //IF A IS FIRST
					if (abs(angle1 - radiansToDegrees(currentPosition1.ang) ) > 1 && !done1S){
						commandVelocity1.angular.z = findAngularVelocity(angle1,startAngle1);
					}
					else { //whes finishes rotation, it stops
						done1S = 1;
						commandVelocity1.angular.z = 0;
						commandVelocity1.linear.x = 0;
					}
					if (abs(angle2 - radiansToDegrees(currentPosition2.ang)) > 1 && !done2S){
						commandVelocity2.angular.z = findAngularVelocity(angle2,startAngle2);
					}
					else {
						done2S = 1;
						commandVelocity2.angular.z = 0;
						commandVelocity2.linear.x = 0;
					}

					if (!isOnlySimulation){ //for real robots
						if (abs(angle1 - radiansToDegrees(currentPosition1Real.ang)) > 5 && !done1E){
							commandVelocity1R.angular.z = findAngularVelocity(angle1,startAngle1);
						}
						else {
							done1E = 1;
							commandVelocity1R.angular.z = 0;
							commandVelocity1R.linear.x = 0;
						}

						if (abs(angle2 - radiansToDegrees(currentPosition2Real.ang)) > 5 && !done2E){
							commandVelocity2R.angular.z = findAngularVelocity(angle2,startAngle2);
						}
						else {
							done2E = 1;
							commandVelocity2R.angular.z = 0;
							commandVelocity2R.linear.x = 0;
						}
					}
					else { done1E = 1;  done2E=1;} //if only simulation is running
			}

			else { //IF B IS FIRST
					if (abs(angle1 - radiansToDegrees(currentPosition2.ang)) > 1 && !done2S){
						commandVelocity2.angular.z = findAngularVelocity(angle1,startAngle2);
					}
					else {
						done2S = 1;
						commandVelocity2.angular.z = 0;
						commandVelocity2.linear.x = 0;
					}
					if (abs(angle2 - radiansToDegrees(currentPosition1.ang)) > 1 && !done1S){
						commandVelocity1.angular.z = findAngularVelocity(angle2,startAngle1);
					}
					else {
						done1S = 1;
						commandVelocity1.angular.z = 0;
						commandVelocity1.linear.x = 0;
					}

					if (!isOnlySimulation){
						if (abs(angle1 - radiansToDegrees(currentPosition2Real.ang)) > 5 && !done2E){
							commandVelocity2R.angular.z = findAngularVelocity(angle1,startAngle2);
						}
						else {
							done2E = 1;
							commandVelocity2R.angular.z = 0;
							commandVelocity2R.linear.x = 0;
						}

						if (abs(angle2 - radiansToDegrees(currentPosition1Real.ang)) > 5 && !done1E){
							commandVelocity1R.angular.z = findAngularVelocity(angle2,startAngle1);
						}
						else {
							done1E = 1;
							commandVelocity1R.angular.z = 0;
							commandVelocity1R.linear.x = 0;
						}
					}
					else { done1E = 1;  done2E=1;}
			}
		}

			if (done1S && done2S && done1E && done2E) {
				//when all robots finish their job in current phase, next phase
				//is triggered with all parameters equal zero
				phase1 = 0;
				phase2 = 1;
				phase3 = 0;
				done1S = 0;
				done2S = 0;
				done1E = 0;
				done2E = 0;
				front = inFront;
				start = 0;
			}


		if (phase2){
			if(deltaAngle>=0){
				angle1 = round(radiansToDegrees(currentAngle) + 90);
				angle2 = round(radiansToDegrees(currentAngle) - 90);
			}
			else {
				angle2 = round(radiansToDegrees(currentAngle) + 90);
				angle1 = round(radiansToDegrees(currentAngle) - 90);
			}
			commandVelocity1.linear.x = linearVelocity;
			commandVelocity2.linear.x = linearVelocity;
			commandVelocity1R.linear.x = linearVelocity;
			commandVelocity2R.linear.x = linearVelocity;
			angle1 = standardizeAngleD(angle1);
			angle2 = standardizeAngleD(angle2);
			double delta1 = angle1 - radiansToDegrees(currentPosition1.ang);
			double delta2 = angle2 - radiansToDegrees(currentPosition1.ang);
			if (abs(delta1) < 1 || abs(delta2) < 1){
				done1S = 1;
				commandVelocity1.angular.z = 0;
				commandVelocity1.linear.x = 0;
			}

			 delta1 = angle1 - radiansToDegrees(currentPosition2.ang);
			 delta2 = angle2 - radiansToDegrees(currentPosition2.ang);
			if (abs(delta1) < 1 || abs(delta2) < 1){
				done2S = 1;
				commandVelocity2.angular.z = 0;
				commandVelocity2.linear.x = 0;
			}
			if (deltaAngle >= 0){
				commandVelocity1.angular.z = 2*linearVelocity/robotLength;
				commandVelocity2.angular.z = 2*linearVelocity/robotLength;
			}
			else{
				commandVelocity1.angular.z = -2*linearVelocity/robotLength;
				commandVelocity2.angular.z = -2*linearVelocity/robotLength;
			}

			if (!isOnlySimulation){
			  delta1 = angle1 - radiansToDegrees(currentPosition1Real.ang);
			  delta2 = angle2 - radiansToDegrees(currentPosition1Real.ang);
			  if (abs(delta1) < 1 || abs(delta2) < 1){
				 	done1E = 1;
				 	commandVelocity1R.angular.z = 0;
				 	commandVelocity1R.linear.x = 0;
				}
			  delta1 = angle1 - radiansToDegrees(currentPosition2Real.ang);
				delta2 = angle2 - radiansToDegrees(currentPosition2Real.ang);
			  if (abs(delta1) < 1 || abs(delta2) < 1){
				  done2E = 1;
				  commandVelocity2R.angular.z = 0;
				  commandVelocity2R.linear.x = 0;
			 }
			 if (deltaAngle >= 0){
				 commandVelocity1R.angular.z = 2*linearVelocity/robotLength;
				 commandVelocity2R.angular.z = 2*linearVelocity/robotLength;
			 }
			 else{
				 commandVelocity1R.angular.z = -2*linearVelocity/robotLength;
				 commandVelocity2R.angular.z = -2*linearVelocity/robotLength;
			 }
			}
			else {done1E = 1;done2E = 1;}


			if (done1S && done2S && done1E && done2E) {
				phase1 = 0;
				phase2 = 0;
				phase3 = 1;
				done1S = 0;
				done2S = 0;
				done1E = 0;
				done2E = 0;
			}


		}

		if(phase3){

			angle1 = round(radiansToDegrees(nextAngle));

			angle1 = standardizeAngleD(angle1);

			angle2 = angle1;

			commandVelocity1.linear.x = 0;
			commandVelocity2.linear.x = 0;
			commandVelocity1R.linear.x = 0;
			commandVelocity2R.linear.x = 0;

			double static startAngle1,startAngle2;
			if (!start){
				start = 1;
				startAngle1 = radiansToDegrees(currentPosition1.ang);
				startAngle2 = radiansToDegrees(currentPosition2.ang);

			}

			if(inFront == 1 || (inFront == 0 && first == 1)){  //IF A IS FIRST

					if (abs(angle1 - radiansToDegrees(currentPosition1.ang)) > 1 && !done1S){
						commandVelocity1.angular.z = findAngularVelocity(angle1,startAngle1);
					}
					else {
						done1S = 1;
						commandVelocity1.angular.z = 0;
						commandVelocity1.linear.x = 0;
					}
					if (abs(angle2 - radiansToDegrees(currentPosition2.ang)) > 1 && !done2S){
						commandVelocity2.angular.z = findAngularVelocity(angle2,startAngle2);
					}
					else {
						done2S = 1;
						commandVelocity2.angular.z = 0;
						commandVelocity2.linear.x = 0;
					}

					if (!isOnlySimulation){
						if (abs(angle1 - radiansToDegrees(currentPosition1Real.ang)) > 5 && !done1E){
							commandVelocity1R.angular.z = findAngularVelocity(angle1,startAngle1);
						}
						else {
							done1E = 1;
							commandVelocity1R.angular.z = 0;
							commandVelocity1R.linear.x = 0;
						}

						if (abs(angle2 - radiansToDegrees(currentPosition2Real.ang)) > 5 && !done2E){
							commandVelocity2R.angular.z = findAngularVelocity(angle2,startAngle2);
						}
						else {
							done2E = 1;
							commandVelocity2R.angular.z = 0;
							commandVelocity2R.linear.x = 0;
						}
					}
					else { done1E = 1;  done2E=1;}
				}

			else{
					if (abs(angle1 - radiansToDegrees(currentPosition2.ang)) > 1 && !done2S){
						commandVelocity2.angular.z = findAngularVelocity(angle1,startAngle2);
					}
					else {
						done2S = 1;
						commandVelocity2.angular.z = 0;
						commandVelocity2.linear.x = 0;
					}
					if (abs(angle2 - radiansToDegrees(currentPosition1.ang)) > 1 && !done1S){
						commandVelocity1.angular.z = findAngularVelocity(angle2,startAngle1);
					}
					else {
						done1S = 1;
						commandVelocity1.angular.z = 0;
						commandVelocity1.linear.x = 0;
					}

					if (!isOnlySimulation){
						if (abs(angle1 - radiansToDegrees(currentPosition2Real.ang)) > 5 && !done2E){
							commandVelocity2R.angular.z = findAngularVelocity(angle1,startAngle2);
						}
						else {
							done2E = 1;
							commandVelocity2R.angular.z = 0;
							commandVelocity2R.linear.x = 0;
						}

						if (abs(angle2 - radiansToDegrees(currentPosition1Real.ang)) > 5 && !done1E){
							commandVelocity1R.angular.z = findAngularVelocity(angle2,startAngle1);
						}
						else {
							done1E = 1;
							commandVelocity1R.angular.z = 0;
							commandVelocity1R.linear.x = 0;
						}
					}
					else { done1E = 1;  done2E=1;}
				}

			if (done1S && done2S && done1E && done2E) {
				phase1 = 1;
				phase2 = 0;
				phase3 = 0;
				done1S = 0;
				done2S = 0;
				done1E = 0;
				done2E = 0;
				start = 0;
				deltaAngleR = 0;
				/*
				   It means that all phases are done because when this line comes,
					 previousAngle = currentAngle - deltaAngleR;
					  previousAngle becomes equal to currentAngle.
				*/


			}

		}

	}


}

double calculateNextAngle(int index){

	/*
		Next Angle is only necessary for phase3. It tells robots which angle they need to have after
		rotation of config. Algorithm is calculating the diffenrece in x and y directions of current
		state and the next, and width atan2 finds angle.
	*/

	if (index < transformedPath.poses.size()-1){
		double y = globalPath.poses[index+1].pose.position.y - globalPath.poses[index].pose.position.y;
		double x = globalPath.poses[index+1].pose.position.x - globalPath.poses[index].pose.position.x;
		return atan2(y,x);
	}
	else{
		double y = globalPath.poses[index].pose.position.y - globalPath.poses[index-1].pose.position.y;
		double x = globalPath.poses[index].pose.position.x - globalPath.poses[index-1].pose.position.x;
		return atan2(y,x);
	}
}

bool computeVelocityCommands(geometry_msgs::Twist& commandVelocity1, geometry_msgs::Twist& commandVelocity2,
														 geometry_msgs::Twist& commandVelocity1R, geometry_msgs::Twist& commandVelocity2R)
{
    //Check if the vehicle has reached the end of path
    double distanceX,distanceY;
    distanceX = transformedPath.poses[transformedPath.poses.size()-1].pose.position.x;
    distanceY = transformedPath.poses[transformedPath.poses.size()-1].pose.position.y;
    if (sqrt(distanceX * distanceX + distanceY * distanceY) < positionTolerance || done == true)
    {
        done = true;
				pathSet = 0;
        commandVelocity1.linear.x = 0;
        commandVelocity1.angular.z = 0;
				commandVelocity2.linear.x = 0;
        commandVelocity2.angular.z = 0;
				commandVelocity1R.linear.x = 0;
        commandVelocity1R.angular.z = 0;
				commandVelocity2R.linear.x = 0;
        commandVelocity2R.angular.z = 0;

        return true;
    }
    if(!pathPruned)
        prunePath();

		//finding state at lookahead distance from current config state
		geometry_msgs::Quaternion quat;
		quat.z = transformedPath.poses[0].pose.orientation.z;
		quat.w = transformedPath.poses[0].pose.orientation.w;
    distanceX = transformedPath.poses[0].pose.position.x;
    distanceY = transformedPath.poses[0].pose.position.y;
		if(pathLength == transformedPath.poses.size())
			currentAngle = 2 * atan2(quat.z,quat.w);   	//calculating next angle at the beginning


    double previousDifference = abs(sqrt(distanceX * distanceX + distanceY * distanceY) - lookaheadDistance);
		int index; //place on the path of desired location
    for(int i = 1; i < transformedPath.poses.size();i++)
    {
        distanceX = transformedPath.poses[i].pose.position.x;
        distanceY = transformedPath.poses[i].pose.position.y;
        double pointDifference = abs(sqrt(distanceX * distanceX + distanceY * distanceY) - lookaheadDistance);

	      if (pointDifference > previousDifference)
	      	{
						index = i;
						quat.z = transformedPath.poses[i].pose.orientation.z;
						quat.w = transformedPath.poses[i].pose.orientation.w;
						previousAngle = currentAngle - deltaAngleR;
						currentAngle = 2 * atan2(quat.z,quat.w);
	          break;
	      	}
		     else
           previousDifference = pointDifference;
		 }

		 //erasing states that robots passed only if robots are not
		 //at the end of path (lookaheadDistance/delta is
		 // lookahead expressed in cell numbers)

		if (transformedPath.poses.size() > lookaheadDistance/delta)
			for(int i = 0; i < index-1; i++)
				{
				transformedPath.poses.erase(transformedPath.poses.begin());
				globalPath.poses.erase(globalPath.poses.begin());
				}
		else
			index = transformedPath.poses.size()-1;

			// for calculating speeds (only if change in angle occurs)
		if(phase1 || phase2 || phase3) nextAngle = calculateNextAngle(index);


		double X = globalPath.poses[index].pose.position.x;
		double Y = globalPath.poses[index].pose.position.y;
		//robotPositions[0] - next state of first robot
		//robotPositions[1] - next state of second robot
		vector<State> robotPositions = findRobotPositions(currentAngle,X,Y);

		//calculating which robot is in front and which is following which point.
		// Algoritm is described in the paper.
		double A1 = euclideanDistance(robotPositions[0],currentPosition1);
		double B1 = euclideanDistance(robotPositions[0],currentPosition2);
		double A2 = euclideanDistance(robotPositions[1],currentPosition1);
		double B2 = euclideanDistance(robotPositions[1],currentPosition2);
		double max1 = max(A1,A2);
		double max2 = max(B1,B2);
		double maxDistance = max(max1,max2);

		if (B1 == maxDistance || B2 == maxDistance)
			inFront = 1;
		else
			inFront = 2;
		if (abs(max1-max2)<positionTolerance) inFront = 0;

		//the robot with lesser distance from point robotPositions[0] follows it
		//and other robot is following other point
		if (B1 < A1){
			State state1 = robotPositions[0];
			robotPositions[0] = robotPositions[1];
			robotPositions[1] = state1;
		}

		//transformation of coordinates back to robot's system
		State point1 = robotPositions[0];
		State point2 = robotPositions[1];
		double x = point1.x - currentPosition1.x;
		double y = point1.y - currentPosition1.y;
		double len = sqrt(x*x + y*y);
		double ang = atan2(x,y) + currentPosition1.ang;
		point1.x = len*sin(ang);
		point1.y = len*cos(ang);

		x = point2.x - currentPosition2.x;
		y = point2.y - currentPosition2.y;
		len = sqrt(x*x + y*y);
		ang =  atan2(x,y) + currentPosition2.ang;
		point2.x = len*sin(ang);
		point2.y = len*cos(ang);

		calculateVelocities(commandVelocity1,point1,commandVelocity2,point2,commandVelocity1R,commandVelocity2R);
		ROS_INFO("%f",euclideanDistance(currentPosition1,currentPosition2)); //printing distance between robots


    return true;
}



int main(int argc, char **argv)
{

	ros::init(argc, argv,"velocity_publisher");
	ros::NodeHandle vel;
	ros::Subscriber m = vel.subscribe("/Alfa/map_loc",1,mapCallback);
	ros::Subscriber cp = vel.subscribe("/currentPosition",1,currentPositionCallback);
  ros::Subscriber pa = vel.subscribe("/path",1,globalPathCallback);

	XmlRpc::XmlRpcValue vsa;
	XmlRpc::XmlRpcValue vsb;
	XmlRpc::XmlRpcValue names;
	vector<string> vehicleNames;
	vel.getParam("/spawn1", vsa);
	vel.getParam("/spawn2", vsb);
	vel.getParam("/robotLength",robotLength);
	vel.getParam("/delta",delta);
	vel.getParam("/timeStamp", timeStamp);
	vel.getParam("/lookahead",lookaheadDistance);
	vel.getParam("/linearVelocity",linearVelocity);
	vel.getParam("/angularVelocity",angularVelocity);
	vel.getParam("/vehicleNames",names);


	linearVelocity = linearVelocity;
	ros::Rate wait(timeStamp);		//frequency of node execution

	//some Xml sorcery
	double A[3],B[3];
	for (int i = 0; i < 3;i++){
		A[i] = static_cast<double>(vsa[i]);
		B[i] = static_cast<double>(vsb[i]);					//casting from XML to CPP
	}
	spawn1.x = A[0];
	spawn1.y = A[1];
	spawn1.ang = A[2];
	spawn2.x = B[0];
	spawn2.y = B[1];
	spawn2.ang = B[2];

	//getting vehicle names
	for (int32_t i = 0; i < names.size(); ++i)
	{
		ROS_ASSERT(names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		string vehicleName = static_cast<string>(names[i]);


		vehicleNames.push_back(vehicleName);
	}

	ros::Publisher vel1 = vel.advertise<geometry_msgs::Twist>("/"+vehicleNames[0]+"/cmd_vel_sim",timeStamp);
	ros::Publisher vel2 = vel.advertise<geometry_msgs::Twist>("/"+vehicleNames[1]+"/cmd_vel_sim",timeStamp);
	ros::Subscriber cp1 = vel.subscribe("/" + vehicleNames[0] + "/simPose",1,currentPositionCallback1);
	ros::Subscriber cp2 = vel.subscribe("/" + vehicleNames[1] + "/simPose",1,currentPositionCallback2);
	ros::Subscriber cp1R = vel.subscribe("/" + vehicleNames[0] + "/pose",1,currentPositionCallback1Real);
	ros::Subscriber cp2R = vel.subscribe("/" + vehicleNames[1] + "/pose",1,currentPositionCallback2Real);
	ros::Publisher vel1R = vel.advertise<geometry_msgs::Twist>("/"+vehicleNames[0]+"/cmd_vel",timeStamp);
	ros::Publisher vel2R = vel.advertise<geometry_msgs::Twist>("/"+vehicleNames[1]+"/cmd_vel",timeStamp);

	while(!resolution || !pathSet){
		ros::spinOnce();
		wait.sleep();
		//waiting for path to be set
	}
	initializePurePursuit();
  while(ros::ok() && pathSet){
		//main program
		ros::spinOnce();
		wait.sleep();
    geometry_msgs::Twist commandVelocity1,commandVelocity2,commandVelocity1R,commandVelocity2R;
    computeVelocityCommands(commandVelocity1,commandVelocity2,commandVelocity1R,commandVelocity2R);
		vel1.publish(commandVelocity1);
    vel2.publish(commandVelocity2);
		vel1R.publish(commandVelocity1R);
    vel2R.publish(commandVelocity2R);
		while(!pathSet){
			if (done)
				ROS_INFO("DONE!!!!!!!!");

			ros::spinOnce();
			wait.sleep();
		}
  }

}
