#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
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

struct Mapa{
	uint8_t* map;
	int width;
	int height;
	double resolution;
	int scalingFactor;
};

Mapa mapGrid;


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
   mapGrid.width = msg->info.width;
	 mapGrid.height = msg->info.height;
	 mapGrid.resolution = msg->info.resolution;
	 mapGrid.scalingFactor = 1;
	uint size = mapGrid.width * mapGrid.height;


	if (!mapGrid.map)
			mapGrid.map = (uint8_t*)malloc(size);

	for(int i = 0; i < mapGrid.height;i++)
		for (int j = 0; j < mapGrid.width;j++)
			mapGrid.map[i * mapGrid.width + j] = msg->data[j * mapGrid.width + i];			// map in nav_msgs is reversed
}

void addForbiddenAreaToMap(Mapa someMap,const double radius){
	State position;
	int r = round(radius / someMap.resolution);
	double distance;
	int minus_r = -r;
	for (int e = 0; e < edgesLength; e++){
		for (int i = minus_r; i < r; i++){
			for (int j = minus_r; j < r; j++){
				position.x = edges[e].x + i;
				position.y = edges[e].y + j;

				distance = euclideanDistance(edges[e],position) * someMap.resolution;
				if (distance < radius )
					mapGrid.map[position.x * mapGrid.width + position.y] = 100;

			}
		}

	}
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
  	//	printScaledMap(*scaledMap);
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
    			if (mapGrid.map[i * mapGrid.width + j])
    				 		str += '#';
    			else
    				 		str += '.';
    		}
    		 myfile << (str.data());
    	}

    	myfile.close();
    }

double robotWidth = 0.5;

int main(int argc, char **argv)
{

	ros::init(argc, argv,"maper");
	ros::NodeHandle mp;
	ros::Subscriber m = mp.subscribe("/Alfa/map_loc",1,mapCallback);
	ros::Publisher map_pub = mp.advertise<nav_msgs::OccupancyGrid::ConstPtr>("/scaledMap",10);
	mapGrid.map = NULL;
  ros::Rate loop_rate(5);
  int i = 0;
  while(ros::ok() && mapGrid.map == NULL && i<100){
    i++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  Mapa scaledMap;
	scaledMap.scalingFactor = (round(0.05/mapGrid.resolution));
	scaledMap.width = mapGrid.width/scaledMap.scalingFactor;
	scaledMap.height = mapGrid.height/scaledMap.scalingFactor;
	scaledMap.resolution = mapGrid.resolution * scaledMap.scalingFactor;
	scaledMap.map = (uint8_t*) malloc(scaledMap.width * scaledMap.height);
	//printMap();
	scaleMap(&scaledMap);

  nav_msgs::OccupancyGrid::ConstPtr mapa;
  mapa->info.resolution = scaledMap.resolution;
  mapa->info.width = scaledMap.width;
  mapa->info.height = scaledMap.height;
  mapa->data = scaledMap.map;

  map_pub.publish(mapa);
  return 0;
}
