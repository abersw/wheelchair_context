/*
 * detected_objects_context.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
 * 
 * Goal for this software:
 * add same calculation software
 * object weighting = maximum weighting value (100%) / times trained (by user)
 * object weighting is capped at 0 and 100%
 * object uniqueness = maximum uniqueness (100%) / instances
 * 
 * notes for calculating uniqueness
 * uniqueness is percentage
 * if you have two rooms and it is located in 1, it's a unique item
 * occurances divided by total rooms
 *  1 / 2 = 0.5 * 100 = 50 - 50% common
 *  2 / 3 = 0.6667 * 100 = 66.66  |  100 - 66.6 = 33.34
 * what about rooms which haven't been trained as extensively? What affect does that have on uniqueness?

 * Now we have the number of objects within the room..., so we can take this into consideration
 * 
 * On second thought all this stuff above could go in the navigation node - as it's part of the decision making process
 * 
 * This needs an input from the user - this is the kitchen
 * Detected objects will then be allocated a room
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "wheelchair_msgs/objectLocations.h"
#include "wheelchair_msgs/objectContext.h"
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

const int DEBUG_doesPkgExist = 0;
const int DEBUG_createFile = 0;
const int DEBUG_main = 1;

struct Objects { //struct for publishing topic
    int id; //get object id from ros msg
    string object_name; //get object name/class
    float object_confidence; //get object confidence

    float point_x; //get transform point x
    float point_y; //get transform point y
    float point_z; //get transform point z

    float quat_x; //get transform rotation quaternion x
    float quat_y; //get transform rotation quaternion y
    float quat_z; //get transform rotation quaternion z
    float quat_w; //get transform rotation quaternion w

    int inLastFrame; //unique to context file - set var to 1 if in last frame
};
struct Objects objectsFileStruct[100000]; //array for storing object data
int totalObjectsFileStruct = 0; //total objects inside struct

struct Context {
    int object_id;
    string object_name;
    float object_confidence;
    int object_detected;

    float object_weighting;
    float object_uniqueness;
    int object_instances;
};

struct Context objectContext[100000]; //struct for storing object context info
int totalObjectContextStruct = 0; //total objects in struct

struct Links {
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

/**
 * Does the wheelchair_dump package exist in the workspace?
 * If it's missing, close down the node safely
 */
std::string doesPkgExist(std::string pkg_name) {
    std::string getPkgPath;
	if (ros::package::getPath(pkg_name) == "") {
		cout << "FATAL:  Couldn't find package " << pkg_name << "\n";
		cout << "FATAL:  Closing node. \n";
        if (DEBUG_doesPkgExist) {
            cout << getPkgPath << endl;
        }
		ros::shutdown();
		exit(0);
	}
    else {
        getPkgPath = ros::package::getPath(pkg_name);
        if (DEBUG_doesPkgExist) {
            cout << getPkgPath << endl;
        }
    }
    return getPkgPath;
}

/**
 * Function to check if file exists in the 'fileName' path, if it doesn't exist create a new one
 *
 * @param pass the path and file name to be created called 'fileName'
 * @return return '1' if file already exists, return '0' if file was missing and has been created
 */
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

/**
 * Main callback function triggered by received ROS topic 
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
    totalObjectsFileStruct = totalObjectsInMsg;
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        objectsFileStruct[isObject].id = obLoc.id[isObject];
        objectsFileStruct[isObject].object_name = obLoc.object_name[isObject];
        objectsFileStruct[isObject].object_confidence = obLoc.object_confidence[isObject];

        objectsFileStruct[isObject].point_x = obLoc.point_x[isObject];
        objectsFileStruct[isObject].point_y = obLoc.point_y[isObject];
        objectsFileStruct[isObject].point_z = obLoc.point_z[isObject];

        objectsFileStruct[isObject].quat_x = obLoc.quat_x[isObject];
        objectsFileStruct[isObject].quat_y = obLoc.quat_y[isObject];
        objectsFileStruct[isObject].quat_z = obLoc.quat_z[isObject];
        objectsFileStruct[isObject].quat_w = obLoc.quat_w[isObject];
    }
}

/**
 * Main callback function triggered by detected objects in frame ROS topic 
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void detectedObjectCallback(const wheelchair_msgs::objectLocations obLoc) {
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
}

/**
 * Last function to save all struct data into files, ready for using on next startup 
 */
void saveAllFiles() {
    cout << "saving all files" << endl;
}

/**
 * Main function that contains ROS info, subscriber callback trigger and while loop to get room name
 *
 * @param argc - number of arguments
 * @param argv - content of arguments
 * @return 0 - end of program
 */
int main (int argc, char **argv) {
    //add code here
    //notes:
    //take UID from publish_objects_location and pass it through here
    //when msg comes through with UID of object - append a room name to the object
    ros::init(argc, argv, "detected_objects_context");
    ros::NodeHandle n;

    wheelchair_dump_loc = doesPkgExist("wheelchair_dump");//check to see if dump package exists
    room_list_loc = wheelchair_dump_loc + dump_context_loc + room_list_name; //concatenate vars to create location of room list
    createFile(room_list_loc); //check to see if file is present, if not create a new one

    ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 10, objectLocationsCallback); //full list of objects
    ros::Subscriber detected_objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 10, detectedObjectCallback); //detected objects in frame
    
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