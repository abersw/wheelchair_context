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

static const int DEBUG_doesPkgExist = 0;
static const int DEBUG_createFile = 0;
static const int DEBUG_contextListToStruct = 1;
static const int DEBUG_detectedObjectCallback = 1;
static const int DEBUG_main = 1;

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

    int inLastFrame = 0; //unique to context file - set var to 1 if in last frame
};
struct Objects objectsFileStruct[100000]; //array for storing object data
int totalObjectsFileStruct = 0; //total objects inside struct
static const int numOfPastFrames = 5;
struct Objects objectsDetectedStruct[numOfPastFrames][1000]; //5 previous frames, 1000 potential objects
int totalObjectsDetectedStruct[numOfPastFrames]; //size of struct for previous 5 frames

struct Context {
    int object_id; //object id
    string object_name; //object name
    float object_confidence; //object confidence from dnn
    int object_detected; //times object has been detected

    float object_weighting; //object weighting result
    float object_uniqueness; //object uniqueness result
    int object_instances; //number of objects in env
};
//object_id,object_name,object_confidence,object_detected,object_weighting,object_uniqueness,object_instances

struct Context objectContext[100000]; //struct for storing object context info
int totalObjectContextStruct = 0; //total objects in struct

//list of file locations
std::string wheelchair_dump_loc;
std::string dump_context_loc = "/dump/context/";
std::string context_list_name = "objects.context";
std::string context_list_loc;

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

void listToContextInfo() {
    //do stuff

}

void contextListToStruct(std::string fileName) {
    std::string objectsDelimiter = ","; //delimiter character is comma
	ifstream FILE_READER(fileName); //open file
    int objectNumber = 0; //iterate on each object
    if (FILE_READER.peek() == std::ifstream::traits_type::eof()) {
        //don't do anything if next character in file is eof
        cout << "file is empty" << endl;
    }
    else {
        std::string line;
        while (getline(FILE_READER, line)) { //go through line by line
            int lineSection = 0; //var for iterating through serialised line
            int pos = 0; //position of delimiter in line
            std::string token;
            while ((pos = line.find(objectsDelimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                //std::cout << token << std::endl;
                line.erase(0, pos + objectsDelimiter.length());
                //deserialise the line sections below:
                if (lineSection == 0) {
                    objectContext[objectNumber].object_id = std::stoi(token); //set id of object from existing id
                }
                else if (lineSection == 1) {
                    objectContext[objectNumber].object_name = token; //set object name
                }
                else if (lineSection == 2) {
                    objectContext[objectNumber].object_confidence = std::stof(token); //set object confidence
                }
                else if (lineSection == 3) {
                    objectContext[objectNumber].object_detected = std::stoi(token); //set times object has been detected
                }
                else if (lineSection == 4) {
                    objectContext[objectNumber].object_weighting = std::stof(token); //set object weighting
                }
                else if (lineSection == 5) {
                    objectContext[objectNumber].object_uniqueness = std::stof(token); //set object uniqueness
                }
                lineSection++; //move to next element in line
            }
            objectContext[objectNumber].object_instances = std::stof(line); //set object instances
            if (DEBUG_contextListToStruct) { //print off debug lines
                cout << "sections in line " << lineSection << endl;
                cout << objectContext[objectNumber].object_id << "," << objectContext[objectNumber].object_name << ", " << objectContext[objectNumber].object_confidence << endl;
                cout << objectContext[objectNumber].object_detected << ", " << objectContext[objectNumber].object_weighting << ", " << objectContext[objectNumber].object_uniqueness << ", " << objectContext[objectNumber].object_instances << endl;
                printSeparator(0);
            }
            objectNumber++; //iterate to next object in list
        }
    }
    //totalObjectsFileStruct = objectNumber; //var to add number of objects in struct
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
    totalObjectContextStruct = totalObjectsFileStruct;
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

        objectContext[isObject].object_id = objectsFileStruct[isObject].id;
        objectContext[isObject].object_name = objectsFileStruct[isObject].object_name;
        objectContext[isObject].object_confidence = objectsFileStruct[isObject].object_confidence;
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
    int detPos = 0; //detection position corresponds with previous frames
    totalObjectsDetectedStruct[detPos] = totalObjectsInMsg;
    for (int detectedObject = 0; detectedObject < totalObjectsDetectedStruct[detPos]; detectedObject++) {
        //add to struct position [0][object number]
        objectsDetectedStruct[detPos][detectedObject].id = obLoc.id[detectedObject];
        objectsDetectedStruct[detPos][detectedObject].object_name = obLoc.object_name[detectedObject];
        objectsDetectedStruct[detPos][detectedObject].object_confidence = obLoc.object_confidence[detectedObject];

        objectsDetectedStruct[detPos][detectedObject].point_x = obLoc.point_x[detectedObject];
        objectsDetectedStruct[detPos][detectedObject].point_y = obLoc.point_y[detectedObject];
        objectsDetectedStruct[detPos][detectedObject].point_z = obLoc.point_z[detectedObject];

        objectsDetectedStruct[detPos][detectedObject].quat_x = obLoc.quat_x[detectedObject];
        objectsDetectedStruct[detPos][detectedObject].quat_y = obLoc.quat_y[detectedObject];
        objectsDetectedStruct[detPos][detectedObject].quat_z = obLoc.quat_z[detectedObject];
        objectsDetectedStruct[detPos][detectedObject].quat_w = obLoc.quat_w[detectedObject];

        //objectsDetectedStruct[detPos][detectedObject].inLastFrame; //don't do anything yet

        if (DEBUG_detectedObjectCallback) {
            cout << 
            objectsDetectedStruct[detPos][detectedObject].id << "," << 
            objectsDetectedStruct[detPos][detectedObject].object_name << "," << 
            objectsDetectedStruct[detPos][detectedObject].object_confidence << "," << 

            objectsDetectedStruct[detPos][detectedObject].point_x << "," << 
            objectsDetectedStruct[detPos][detectedObject].point_y << "," << 
            objectsDetectedStruct[detPos][detectedObject].point_z << "," << 

            objectsDetectedStruct[detPos][detectedObject].quat_x << "," << 
            objectsDetectedStruct[detPos][detectedObject].quat_y << "," << 
            objectsDetectedStruct[detPos][detectedObject].quat_z << "," << 
            objectsDetectedStruct[detPos][detectedObject].quat_w << endl; 
        }
    }
    //finished adding detected data to pos 0 in 2d array

    if (totalObjectsDetectedStruct[detPos+1] == 0) {
        //no history to compare with, therefore do all the calculation stuff
        if (DEBUG_detectedObjectCallback) {
            cout << "nothing in struct pos " << detPos+1 << endl;
        }
        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //run through entire objects struct
            //do stuff
        }
    }
    else {
        //history exists, therefore compare with history to see if object was in previous frame
        if (DEBUG_detectedObjectCallback) {
            cout << "data exists in struct pos " << detPos+1 << endl;
        }
    }
}

/**
 * Function to save all context training info, ready for using on next startup 
 */
void contextInfoToList() {
    cout << "saving all files" << endl;
}

/**
 * Last function to save all struct data into files, ready for using on next startup 
 */
void contextStructToList() {
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
    //take UID from publish_objects_location and pass it through here
    //when msg comes through with ID of object - append a room name to the object
    ros::init(argc, argv, "detected_objects_context");
    ros::NodeHandle n;

    wheelchair_dump_loc = doesPkgExist("wheelchair_dump");//check to see if dump package exists
    context_list_loc = wheelchair_dump_loc + dump_context_loc + context_list_name; //concatenate vars to create location of room list
    createFile(context_list_loc); //check to see if file is present, if not create a new one
    contextListToStruct(context_list_loc); //add list to struct

    ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 10, objectLocationsCallback); //full list of objects
    ros::Subscriber detected_objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 10, detectedObjectCallback); //detected objects in frame
    
    ros::Rate rate(10.0);
    while(ros::ok()) {
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    //contextStructToList();
    return 0;
}