/*
 * detected_objects_context.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "wheelchair_msgs/objectLocations.h"
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

const int DEBUG_createFile = 1;
const int DEBUG_main = 1;

struct Objects {
    int object_id;
    string object_name;
    int room_id;
    string room_name;
};

std::string userRoomName;

//list of file locations
std::string wheelchair_dump_loc;
std::string dump_context_loc = "/dump/context/";
std::string room_list_name = "room.list";
std::string room_list_loc;

//function for printing space sizes
void printSeparator(int spaceSize) {
	if (spaceSize == 0) {
		printf("--------------------------------------------\n");
	}
	else {
		printf("\n");
		printf("--------------------------------------------\n");
		printf("\n");
	}
}

//does the wheelchair dump package exist in the workspace?
void doesWheelchairDumpPkgExist() {
	if (ros::package::getPath("wheelchair_dump") == "") {
		cout << "FATAL:  Couldn't find package 'wheelchair_dump' \n";
		cout << "FATAL:  Closing training_context node. \n";
		printSeparator(1);
		ros::shutdown();
		exit(0);
	}
}

//create a file
int createFile(std::string fileName) { //if this doesn't get called, no file is created
    if (DEBUG_createFile) {
        printf("DEBUG: createFile()\n");
    }
	std::ifstream fileExists(fileName);

	if (fileExists.good() == 1) {
		//File exists
        if (DEBUG_createFile) {
            printf("Weighting file exists\n");
        }
		//cout << fileName;
		return 1;
	}
	else {
		//File doesn't exist
        if (DEBUG_createFile) {
            printf("Weighting file doesn't exist\n");
            printf("creating new file\n");
        }
		ofstream NEW_FILE (fileName);
		NEW_FILE.close();
		//cout << fileName;
		return 0;
	}
}

void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
}

void saveAllFiles() {
    cout << "saving all files" << endl;
}

int main (int argc, char **argv) {
    //add code here
    //notes:
    //take UID from publish_objects_location and pass it through here
    //when msg comes through with UID of object - append a room name to the object
    ros::init(argc, argv, "detected_objects_context");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 10, objectLocationsCallback);
    std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");

	room_list_loc = wheelchair_dump_loc + dump_context_loc + room_list_name;
    createFile(room_list_loc);
    
    ros::Rate rate(10.0);
    while(ros::ok()) {
        n.getParam("/wheelchair_robot/param/user/room_name", userRoomName);
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    saveAllFiles();
    return 0;
}