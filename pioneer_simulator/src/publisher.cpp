#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
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


/*
	A Node which finds path. From loadMaker.cpp it takes definition of config state
	and uses it to compute optimal path on a given map.
*/
using namespace std;


struct Mapa{
	//struct which contains map and informations about map
	uint8_t* map;
	int width;
	int height;
	double resolution;
	int scalingFactor;

	int position(int i, int j){
		if (this->width >= this->height)
			return j * this->width + i;
		else
			return j * this->height + i;
	}

	void alloc(){
		if (this->width >= this->height)
			this->map = (uint8_t*)malloc(this->width * this->width);
		else
			this->map = (uint8_t*)malloc(this->height * this->height);
	}
};

Mapa mapGrid; //map taken from map server

struct State{
	//definition of state as described in paper
	int x,y;
	int ang;

	//a lot of operators had to be overloaded
	friend bool operator< (State const& state1, State const& state2){
    return ((state1.x*mapGrid.width + state1.y)*360 + state1.ang < (state2.x*mapGrid.width + state2.y)*360 + state2.ang); //some hash function (works very well)
	}

	friend bool operator> (State const& state1, State const& state2){
		return ((state1.x*mapGrid.width + state1.y)*360 + state1.ang > (state2.x*mapGrid.width + state2.y)*360 + state2.ang);
	}

	friend bool operator== (State const& state1, State const& state2){
    return ((state1.x*mapGrid.width + state1.y == state2.x*mapGrid.width + state2.y) && state1.ang == state2.ang);
	}

	friend bool operator!= (State const& state1, State const& state2){
    return ((state1.x*mapGrid.width + state1.y != state2.x*mapGrid.width + state2.y) || state1.ang != state2.ang);
	}

	 State operator= (const State& state){
		this->x = state.x;
		this->y = state.y;
		this->ang = state.ang;

	}

};
vector<State> edges; //a vector of all edges on the given map

const double PI = 3.1415;
const double SQRT2 = sqrt(2);


State currentPosition, goalPosition;
double robotWidth,robotLength,delta,safety; //parameters from launch file


struct Successor{
	//used for A* algorithm
	State state;
	double cost;

	friend bool operator== (Successor const& succ1, Successor const& succ2){
    return (succ1.state == succ2.state  &&  succ1.cost == succ2.cost);
	}

	friend bool operator!= (Successor const& succ1, Successor const& succ2){
		return (succ1.state != succ2.state  ||  succ1.cost != succ2.cost);

	}

};


class Node{
	/*
		This class if used by A* algorithm. Objects of this class will be organised
		in form of tree.
	*/
private:
	State state;
	Node* parent;
	double cost,heuristic;


public:
	double priority;
	friend bool operator== (Node const& node1, Node const& node2){
    return (node1.priority == node2.priority);
	}

	friend bool operator!= (Node const& node1, Node const& node2){
    return (node1.priority != node2.priority);
	}

	friend bool operator< (Node const& node1, Node const& node2){
    return (node1.priority < node2.priority);
	}

	friend bool operator> (Node const& node1, Node const& node2){
    return (node1.priority > node2.priority);
	}

	friend bool operator<= (Node const& node1, Node const& node2){
    return (node1.priority <= node2.priority);
	}

	friend bool operator>= (Node const& node1, Node const& node2){
    return (node1.priority >= node2.priority);
	}

	//constructor
	Node(State st,Node* par=NULL,double costFn = 0,double prior = 0,double heur = 0){
		state = st;
		parent = par;
		cost = costFn;
		heuristic = heur;
		priority = prior;
	}

	int isRoot(){
		if (this->parent == NULL) return 1;
		else return 0;
	}

	vector<State> backtrack(){
		//used to climb the tree and save the path
	  vector<State> states;
		Node* node = this;
		if (node->isRoot())
			return states;

    states = node->parent->backtrack();
    states.push_back(node->state);
        return states;
	}

//getters
	State getState(){
		return this->state;
	}


	double getPriority(){
		return priority;
	}

	Node* getParent(){
		return this->parent;
	}

	double getCost(){
		return this->cost;
	}

	double getHeuristic(){
		return this->heuristic;
	}
};

struct node_comparison
{
		//this struct is used to compare objects from class Node
		//std::map is using it to sort priority queue
    bool operator () ( const Node* a, const Node* b ) const
    {
        return a->priority > b->priority;
    }
};




double euclideanDistance(State position,State goal){
	double ydist,xdist;
	ydist = (goal.y - position.y);
	xdist = (goal.x - position.x);
	return sqrt(pow(ydist,2) + pow(xdist,2));
}

double manhattanDistance(State position,State goal){
	double ydist,xdist;
	ydist = (goal.y-position.y);
	xdist = (goal.x-position.x);
	return abs(ydist) + abs(xdist);
}
void printStateVector(vector<State>&vec){
	for(int i = 0;i < vec.size();i++)
		ROS_INFO("%d %d %d    %d.",vec[i].x,vec[i].y,vec[i].ang,i);

}


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
   mapGrid.width = msg->info.width;
	 mapGrid.height = msg->info.height;
	 mapGrid.resolution = msg->info.resolution;
	 mapGrid.scalingFactor = 1;

	if (!mapGrid.map)
			mapGrid.alloc();

	//taking map from mapserver
	for(int i = 0; i < mapGrid.height;i++)
		for (int j = 0; j < mapGrid.width;j++)
				mapGrid.map[mapGrid.position(i,j)] = msg->data[mapGrid.position(i,j)];			// map in nav_msgs is reversed

}


void currentPositionCallback(const std_msgs::Int32MultiArray &pose){

	currentPosition.x = int(pose.data[0]);
	currentPosition.y = int(pose.data[1]);
	int ang = pose.data[2];

	//discretizing the angle
	if (ang < -160)
		currentPosition.ang = -180;
	else if (ang < -115)
		currentPosition.ang = -135;
	else if (ang < -70)
		currentPosition.ang = -90;
	else if (ang < -25)
		currentPosition.ang = -45;
	else if (ang < 20)
		currentPosition.ang = 0;
	else if(ang < 65)
		currentPosition.ang = 45;
	else if(ang < 110)
		currentPosition.ang = 90;
	else if(ang < 155)
		currentPosition.ang = 135;
	else
		currentPosition.ang = 180;

 }

void goalPositionCallback(const std_msgs::Float32MultiArray &pose){
	goalPosition.x = int(pose.data[0] / mapGrid.resolution);
	goalPosition.y = int(pose.data[1] / mapGrid.resolution);
	goalPosition.ang = pose.data[2];

}


void printMap(){

	string str;
	ofstream myfile;
	myfile.open ("/home/luka/ROS/zavrsni/src/pioneer_simulator/launch/map.txt",ios::trunc);
	if (myfile.is_open())
		ROS_INFO("Map loaded!");
	for (int i = 0;i < mapGrid.height;i++){
			myfile <<  "\n";
			str = "";
		for (int j = 0;j < mapGrid.width;j++){
			if (mapGrid.map[i*mapGrid.width+j])
				str += '#';

			else
				str += '.';

			}
			 myfile << (str.data());
		}
		myfile.close();
}


void printScaledMap(Mapa scaledMap){

	string str;
	ofstream myfile;
	myfile.open ("/home/luka/ROS/zavrsni/src/pioneer_simulator/launch/scaledMap.txt",ios::trunc);
	if (myfile.is_open())
		ROS_INFO("Map loaded!");
	for (int i = 0; i < scaledMap.height;i++){
		myfile <<  "\n";
		str = "";
		for (int j = 0;j < scaledMap.width;j++){
			if (scaledMap.map[i * scaledMap.width + j])
				str += '#';
 			else
				str += '.';
		}
		myfile << (str.data());
	}
	myfile.close();
	}

void scaleMap(Mapa* scaledMap){
	/*
		This function scales map. It makes square cells on the map of length
		scalingFactor (every cell contains scalingFactor smaller cells of original map).
		Then it counts number of forbidden states in each cell, and if they are in most,
		cell is declared forbidden.
	*/

	for (int i = 0;i < scaledMap->width * scaledMap->height;i++)
		scaledMap->map[i] = 0;
	for (int i = 0;i < mapGrid.height;i++){
		int k = i / scaledMap->scalingFactor;

		for (int j = 0; j < mapGrid.width;j++){
			int l = j / scaledMap->scalingFactor;
			if (mapGrid.map[i * mapGrid.width + j])
				scaledMap->map[k * scaledMap->width + l]++;      //counting black spots
			}

		}

	for (int i = 0;i < scaledMap->height;i++)
		for (int j = 0;j < scaledMap->width;j++){
			if (scaledMap->map[i * scaledMap->width + j] >= scaledMap->scalingFactor * scaledMap->scalingFactor / 5)
				scaledMap->map[i * scaledMap->width + j] = 100;
			else
				scaledMap->map[i * scaledMap->width + j] = 0;
		}
		//printScaledMap(*scaledMap);
	}

	void addForbiddenAreaToMap(const double radius){

		/*
			Function is going through all edges of the map and adds forbidden area
			around it of radius given to function.
		*/
		State position;
		int r = round(radius / mapGrid.resolution);
		double distance;
		int minus_r = -r;
		for (int e = 0; e < edges.size(); e++){
			for (int i = minus_r; i < r; i++){
				for (int j = minus_r; j < r; j++){
					position.x = edges[e].x + i;
					position.y = edges[e].y + j;
					if(position.y < mapGrid.height && position.y > 0 &&
						 position.x < mapGrid.width  && position.x >0){
							 distance = euclideanDistance(edges[e],position) * mapGrid.resolution;
							 if (distance < radius )
								mapGrid.map[position.y * mapGrid.width + position.x] = 100;
					}
				}
			}

		}
	}







vector<State> findCorners(State state){
	/*
		Function finds corners of config in current state in diagonal position.
		Config length and width are not changing so it can be hardcoded.
	*/
	vector<State> corners;
	State s;
	s.ang = state.ang;
	double l = (SQRT2 / 4) * robotLength;
	double w = (SQRT2 / 4) * robotWidth;
	if ( state.ang == -45 || state.ang == 135){
		s.x = (-l - w)/delta;
		s.y = (l - w)/delta;
		corners.push_back(s);

		s.x = (-l + w)/delta;
		s.y = (l + w)/delta;
		corners.push_back(s);

		s.x = (+l - w)/delta;
		s.y = (-l - w)/delta;
		corners.push_back(s);

		s.x = (+l + w)/delta;
		s.y = (-l + w)/delta;
		corners.push_back(s);
	}

	else if ( state.ang == -135 || state.ang == 45){
		s.x = (-l - w)/delta;
		s.y = (-l + w)/delta;
		corners.push_back(s);

		s.x = (l - w)/delta;
		s.y = (l + w)/delta;
		corners.push_back(s);

		s.x = (-l + w)/delta;
		s.y = (-l - w)/delta;
		corners.push_back(s);

		s.x = (l + w)/delta;
		s.y = (l - w)/delta;
		corners.push_back(s);
	}

	for (int i = 0;i<corners.size();i++){
		corners[i].x += state.x;
		corners[i].y += state.y;
	}
	return corners;
}

vector<State> makeLoad(State state){
	/*
		Function makes vector of states which represent edges of the config.
		Again, it is hardcoded.
		KEEP CALM AND HARD CODE!!
	*/
	vector<State> loadSpots;
	State spot;
	int length = robotLength / delta;
	int width = robotWidth / delta;
	spot.ang = state.ang;
	int xiter,yiter;

	if (state.ang == 0 || state.ang == 90 || state.ang == -90 || state.ang == 180){
		if (state.ang == 90 || state.ang == -90){
			yiter = length;
			xiter = width;
		}
		else{
			yiter = width;
			xiter = length;
		}

		for (int i = -yiter/2; i <= yiter/2;i++){
			spot.x = state.x - xiter/2 ;
			spot.y = state.y + i;
			loadSpots.push_back(spot);
			spot.x = state.x + xiter/2 ;
			loadSpots.push_back(spot);
		}
		for (int i = -xiter/2; i <= xiter/2;i++){
			spot.y = state.y - yiter/2 ;
			spot.x = state.x + i;
			loadSpots.push_back(spot);
			spot.y = state.y + yiter/2 ;
			loadSpots.push_back(spot);
		}
	}

	else{
		vector<State> corners;
		State start,end;
		corners = findCorners(state);
		start = corners[0];
		loadSpots.push_back(start);
		if (state.ang == 45 || state.ang == -135 || state.ang == -45 || state.ang == 135){

			start = corners[0];
			end = corners[1];
			while (start.x != end.x && start.y != end.y){
				start.x += 1;
				start.y += 1;
				loadSpots.push_back(start);
			}

			start = corners[0];
			end = corners[2];
			while (start.x != end.x && start.y != end.y){
				start.x += 1;
				start.y -= 1;
				loadSpots.push_back(start);
			}

			start = corners[3];
			end = corners[1];
			loadSpots.push_back(start);
			while (start.x != end.x+1 && start.y != end.y-1){
				start.x -= 1;
				start.y += 1;
				loadSpots.push_back(start);
			}

			start = corners[3];
			end = corners[2];
			while (start.x != end.x+1 && start.y != end.y+1){
				start.x -= 1;
				start.y -= 1;
				loadSpots.push_back(start);
			}
		}
	}
	return loadSpots;

}



void printSuccessors(vector<Successor> successor){

	for(int i = 0;i < successor.size();i++){

	ROS_INFO("(%d %d %d) %f",successor[i].state.x,successor[i].state.y,
																successor[i].state.ang,successor[i].cost);
	}
}

void print_queue(priority_queue<Node*, vector<Node*>, node_comparison > q) {
	 while(!q.empty()) {
			 Node *n = q.top();
			 q.pop();
			 ROS_INFO("%f %f", n->getHeuristic(), n->getPriority() );

	 }
	 ROS_INFO("\n");
}




vector<Successor> getSuccessors(Mapa someMap,State current){

	vector<string> normal,diagonal,rotational;
	vector<Successor> successor;
	vector<State> loadSpots;
	Successor succ;
	State state;
	int x,y;

//PUSHING TRANSLATION SUCCESSORS


// You are not expected to understand this
	for(int i = 0;i < 8;i++){
		if (i < 4 )
			if (i % 2 == 0){
				succ.cost = SQRT2;
				}
			else{
				succ.cost = 1;
				}
		else
			if (i % 2 == 0){
				succ.cost = 1;
				}
			else{
				succ.cost = SQRT2;
	}

	/*
			-WHAT KIND OF WITCHCRAFT IS THIS?!?
				-WITCH PLESE
	*/

	if (i == 4)
		y = 1;
	else if (i>4)
		y = i % 5 -1;
	else
		y = i % 3 -1;

	if (i<3)
		x = -1;
	else if (i>=3 && i<5)
	 	x = 0;
	else
		x = 1;

	state.x = current.x + x;
	state.y = current.y + y;
	state.ang = current.ang;
	succ.state = state;

	int put = 1;

	loadSpots = makeLoad(state); // making load to test if any edge is in forbidden state
	for (int j = 0;j < loadSpots.size();j++)
		if (someMap.map[loadSpots[j].y * someMap.width + loadSpots[j].x] != 0) {
			put = 0;
			break;
		}


	if (put)
		successor.push_back(succ);

	}

	//PUSHING ROTATIONAL SUCCESSORS
	for(int i = 0; i < 2;i++){
		state = current;
		if (!i)
			if (state.ang == 180)
				state.ang = -135;
			else
				state.ang += 45;
		else
			if (state.ang == -135)
				state.ang = 180;  //there can't be an angle of -180!!!
			else if (state.ang == -180)
				state.ang = 135;
			else
				state.ang -= 45;
		succ.state = state;
		succ.cost = (robotLength * (SQRT2 / 4) / delta) * 100;

		int put = 1;
		loadSpots = makeLoad(state);
		for (int j = 0;j < loadSpots.size();j++)
			if (someMap.map[loadSpots[j].y * someMap.width + loadSpots[j].x] != 0) {
				put = 0;
				break;
			}

		if (put)
			successor.push_back(succ);

		}

	return successor;
}


vector<State> aStarSearch(Mapa &someMap,State current, State goal){
	/*
		An A* algorithm is described in the paper
	*/
	priority_queue<Node*, vector<Node*>, node_comparison > queue;
	vector<Successor> successors;
	Node *node = (new Node(current));
	queue.push(node);
	map<State,double> opened, closed;
	opened[current] = 0;

	int it = 0; // used to print how many stats have beed expanded
  while (!queue.empty()){
		Node *position = queue.top();
		queue.pop();
		opened.erase(position->getState());


		State s1 = position->getState();
		State s2 = s1;

		if (s1.ang == 0 || s1.ang == 180 ){
			s1.ang = 180;
			s2.ang = 0;
		}
		else if (s1.ang > 0)
			s2.ang = s1.ang - 180;
		else
			s2.ang = s1.ang + 180;

		if (s1 == goal || s2 == goal)
			return position->backtrack();

		closed[position->getState()] = position->getCost();
		successors = getSuccessors(someMap,position->getState());
		//printSuccessors(successors);
		it++;
		if (it % 10000 == 0) ROS_INFO("%d",it);

		for (int i = 0;i < successors.size();i++){

 			double heuristic = euclideanDistance(successors[i].state, goal);
			double priority = heuristic + successors[i].cost + position->getCost();

			int succInClosed = 0, succInOpened = 0;
			if (closed.count(successors[i].state) > 0){
				succInClosed = 1;
				}

			if (opened.count(successors[i].state) > 0){
				succInOpened = 1;
				}

				if (succInClosed){

					if(successors[i].cost + position->getCost() < closed[successors[i].state]){
						closed[successors[i].state] = successors[i].cost + position->getCost();
						Node *newPosition = (new Node(successors[i].state, position,
																					successors[i].cost + position->getCost(), priority, heuristic));
						queue.push(newPosition);
						opened[successors[i].state] = successors[i].cost + position->getCost();
						continue;

					}
					else
						continue;
				}

				if (succInOpened){
					if(successors[i].cost + position->getCost() < opened[successors[i].state]){
						opened[successors[i].state] = successors[i].cost + position->getCost();
						Node *newPosition = (new Node(successors[i].state, position,
																					successors[i].cost + position->getCost(), priority, heuristic));
						queue.push(newPosition);
						continue;
					}
					else
						continue;
				}

				Node *newPosition = (new Node(successors[i].state, position,
																			successors[i].cost + position->getCost(), priority, heuristic));
				queue.push(newPosition);
				opened[successors[i].state] = successors[i].cost + position->getCost();
		}
	}
	ROS_INFO("CAN'T FIND ANY WAY!!!");
	vector<State> v;
	return v;
}



void findEdges(){
	/*
		Function is finding edges. It is searching for a state which is forbidden
		but on at least one side is free.
	*/
	State position;
	for (int i = 1;i < mapGrid.height-1;i++){
		for(int j = 1;j < mapGrid.width-1;j++){
					if (mapGrid.map[i*mapGrid.width+j]){
						if (mapGrid.map[i*mapGrid.width+j+1]==0||
							 	    mapGrid.map[i*mapGrid.width+j-1]==0||
							      mapGrid.map[(i+1)*mapGrid.width+j]==0||
							      mapGrid.map[(i-1)*mapGrid.width+j]==0){
								position.x = j;
								position.y = i;
								edges.push_back(position);
						}
					}

				}
			}
}







void printPath(nav_msgs::Path path){
	for(int i = 0;i < path.poses.size();i++){
		geometry_msgs::Quaternion quat;
		quat.z = path.poses[i].pose.orientation.z;
		quat.w = path.poses[i].pose.orientation.w;
		double ang = 2 * atan2(quat.z,quat.w);
		ROS_INFO("%f %f %d", path.poses[i].pose.position.x,path.poses[i].pose.position.y,int(ang *180/PI));
	}
}

vector<geometry_msgs::PoseStamped> pathPoseConstruction(vector<State> states,int scalingFactor){

	/*
			Rewriting path from config states to PoseStamped. A* often returns path which
			rotates config at the last state, but because of positionTolerance it does not
			get executed. Because of that, at the end angles are rewriten as you can see.
	*/
	geometry_msgs::Point nextPosition;
	geometry_msgs::Quaternion nextQuaternion;
	geometry_msgs::PoseStamped nextPose;
	vector <geometry_msgs::PoseStamped> poseVector;
	for (int i = 0; i < states.size();i++)
		{
			nextPosition.x = states[i].x * scalingFactor * mapGrid.resolution;
			nextPosition.y = states[i].y * scalingFactor * mapGrid.resolution;
			//angles have a period of 180, so it is the same (only less to worry about this way)
			//angles of config  can only be 0,45,90 or 135 degrees
			if (states[i].ang < 0)
				states[i].ang+=180;
			if (states[i].ang == 180)
				states[i].ang = 0;

			nextPosition.z = 0;

			nextQuaternion.x = 0;
			nextQuaternion.y = 0;
			nextQuaternion.z = tan(static_cast<double>(states[i].ang * PI/180)/2);
			nextQuaternion.w = 1;

			nextPose.pose.position = nextPosition;
			nextPose.pose.orientation = nextQuaternion;
			nextPose.header.frame_id = "map";
			nextPose.header.stamp = ros::Time::now();

			poseVector.push_back(nextPose);

	}
			poseVector[states.size()-4].pose.orientation.z = poseVector[states.size()-2].pose.orientation.z;
			poseVector[states.size()-3].pose.orientation.z = poseVector[states.size()-2].pose.orientation.z;
			poseVector[states.size()-2].pose.orientation.z = poseVector[states.size()-1].pose.orientation.z;
	return poseVector;
}




int main(int argc, char **argv)
{

	ros::init(argc, argv,"topic_publisher");
	ros::NodeHandle pub;
	ros::Subscriber m = pub.subscribe("/Alfa/map_loc",1,mapCallback);
	ros::Subscriber cp = pub.subscribe("/currentPosition",1,currentPositionCallback);
	ros::Subscriber g = pub.subscribe("/goalPosition",1,goalPositionCallback);
	ros::Publisher path_publisher = pub.advertise<nav_msgs::Path>("/path",10);
	ros::Rate loop_rate(10);
	ros::Rate wait(0.5);
	nav_msgs::Path path;
	mapGrid.map = NULL;
	pub.getParam("/robotWidth",robotWidth);
	pub.getParam("/delta",delta);
	pub.getParam("/safety",safety);
	wait.sleep();
	while (ros::ok() && (!mapGrid.map || !currentPosition.x || !robotLength )){
			//waiting for parameters and map
			ros::spinOnce();
			loop_rate.sleep();
			pub.getParam("/robotLength",robotLength);
			continue;
	}
	robotLength += robotWidth/2; //expand by width/2 because robotLength is load length

	/*
			Map has to be scaled so that path finding can be faster. One cell of scaled
			map has dimension delta which is given in launch file.
	*/

	Mapa scaledMap;
	scaledMap.scalingFactor = (round(delta/mapGrid.resolution));
	scaledMap.width = mapGrid.width/scaledMap.scalingFactor;
	scaledMap.height = mapGrid.height/scaledMap.scalingFactor;
	scaledMap.resolution = mapGrid.resolution * scaledMap.scalingFactor;
	scaledMap.alloc();
	findEdges();
	addForbiddenAreaToMap(safety);

	scaleMap(&scaledMap);
	//printMap();

	int found = 0;
	vector<State> loadSpots;
	vector<State> pathStates;
	vector<State> scaledPositions;

	State scaledGoalPosition,scaledCurrentPosition;

	while (ros::ok())
	{
		//main program
		State goalPosition_1 = goalPosition;
		int newGoal = 0;
		ros::spinOnce();
		loop_rate.sleep();

		if(goalPosition_1 != goalPosition) newGoal = 1; // if new goal is set
		if (path.poses.size()){
			path_publisher.publish(path);
		}
		if(!newGoal) continue;

		scaledGoalPosition.x = goalPosition.x / scaledMap.scalingFactor;
		scaledGoalPosition.y = goalPosition.y / scaledMap.scalingFactor;
		scaledGoalPosition.ang = goalPosition.ang;
		scaledCurrentPosition.x = currentPosition.x / scaledMap.scalingFactor;
		scaledCurrentPosition.y = currentPosition.y / scaledMap.scalingFactor;
		scaledCurrentPosition.ang = currentPosition.ang;

		if ((goalPosition.y >= mapGrid.height || goalPosition.x >= mapGrid.width)){
			ROS_INFO("GIVE GOAL INSIDE THE (%d,%d)",int(mapGrid.width*mapGrid.resolution),int(mapGrid.height*mapGrid.resolution));
			continue;
			}

		int put = 1;
		loadSpots = makeLoad(scaledGoalPosition);
		for (int j = 0;j < loadSpots.size();j++)
			if (scaledMap.map[loadSpots[j].y * scaledMap.width + loadSpots[j].x] != 0) {
				put = 0;
				break;
			}
			if (!put){
				 ROS_INFO("CAN'T GET THERE!!!");
				 continue;
			 }

			else found = 0;

		if (!found){


			pathStates = aStarSearch(scaledMap,scaledCurrentPosition,scaledGoalPosition); //A* on scaled map
			if (!pathStates.size()) continue;
			path.poses = pathPoseConstruction(pathStates,scaledMap.scalingFactor);
			path.header.frame_id = "map";
			path.header.stamp = ros::Time::now();
			found = 1;
			printPath(path);
		}

	}
	free(mapGrid.map);
	free(scaledMap.map);
	return 0;
}
