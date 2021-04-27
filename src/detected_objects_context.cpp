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
static const int DEBUG_listToContextInfo = 0;
static const int DEBUG_contextListToStruct = 0;
static const int DEBUG_detectedObjectCallback = 1;
static const int DEBUG_contextInfoToList = 0;
static const int DEBUG_contextStructToList = 0;
static const int DEBUG_main = 0;

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

struct TrainingInfo {
    int times_trained;
    int max_weighting = 100;
    int min_weighting = 0;
};
struct TrainingInfo trainingInfo;

ros::Publisher *ptr_object_context;

//list of file locations
std::string wheelchair_dump_loc; //location of wheelchair_dump package
std::string dump_context_loc = "/dump/context/"; //location of context dir in wheelchair_dump
std::string context_list_name = "objects.context"; //name of object context file
std::string context_info_name = "info.context"; //name of context training info file
std::string context_list_loc; //full path to object context file
std::string context_info_loc; //full path to context training info file

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
 * Function to add training session info from param 'fileName' path, start assigning info from each line of file
 *
 * @param pass the path and file name to be created called 'fileName'
 */
void listToContextInfo(std::string fileName) {
	ifstream FILE_READER(fileName); //open file
    int lineNumber = 0; //iterate on each line
    if (FILE_READER.peek() == std::ifstream::traits_type::eof()) {
        //don't do anything if next character in file is eof
        cout << "file is empty" << endl;
        trainingInfo.times_trained = 1;
    }
    else {
        std::string line;
        while (getline(FILE_READER, line)) { //go through line by line
            //times trained read in and assign to struct
            if (lineNumber == 0) {
                //if line is 0, must be times trained
                int getTimesTrained = std::stoi(line);
                trainingInfo.times_trained = getTimesTrained + 1; //add one to times trained on startup
                if (DEBUG_listToContextInfo) {
                    cout << "training session is " << trainingInfo.times_trained << endl;
                }
            }
            lineNumber++; //iterate to next line
        }
    }
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
    totalObjectContextStruct = objectNumber; //var to add number of objects in struct
}

/**
 * Main callback function triggered by received ROS topic 
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
    totalObjectsFileStruct = totalObjectsInMsg; //set message total objects to total objects in file struct
    totalObjectContextStruct = totalObjectsFileStruct; //set object context struct size to replicate size of object struct
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire msg topic array
        objectsFileStruct[isObject].id = obLoc.id[isObject]; //assign object id to struct
        objectsFileStruct[isObject].object_name = obLoc.object_name[isObject]; //assign object name to struct
        objectsFileStruct[isObject].object_confidence = obLoc.object_confidence[isObject]; //assign object confidence to struct

        objectsFileStruct[isObject].point_x = obLoc.point_x[isObject]; //assign object vector point x to struct
        objectsFileStruct[isObject].point_y = obLoc.point_y[isObject]; //assign object vector point y to struct
        objectsFileStruct[isObject].point_z = obLoc.point_z[isObject]; //assign object vector point z to struct

        objectsFileStruct[isObject].quat_x = obLoc.quat_x[isObject]; //assign object quaternion x to struct
        objectsFileStruct[isObject].quat_y = obLoc.quat_y[isObject]; //assign object quaternion y to struct
        objectsFileStruct[isObject].quat_z = obLoc.quat_z[isObject]; //assign object quaternion z to struct
        objectsFileStruct[isObject].quat_w = obLoc.quat_w[isObject]; //assign object quaternion w to struct

        objectContext[isObject].object_id = objectsFileStruct[isObject].id; //assign object id to context struct
        objectContext[isObject].object_name = objectsFileStruct[isObject].object_name; //assign object name to context struct
        objectContext[isObject].object_confidence = objectsFileStruct[isObject].object_confidence; //assign object confidence to context struct
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
        objectsDetectedStruct[detPos][detectedObject].id = obLoc.id[detectedObject]; //assign object id to struct
        objectsDetectedStruct[detPos][detectedObject].object_name = obLoc.object_name[detectedObject]; //assign object name to struct
        objectsDetectedStruct[detPos][detectedObject].object_confidence = obLoc.object_confidence[detectedObject]; //assign object confidence to struct

        objectsDetectedStruct[detPos][detectedObject].point_x = obLoc.point_x[detectedObject]; //assign object vector point x to struct
        objectsDetectedStruct[detPos][detectedObject].point_y = obLoc.point_y[detectedObject]; //assign object vector point y to struct
        objectsDetectedStruct[detPos][detectedObject].point_z = obLoc.point_z[detectedObject]; //assign object vector point z to struct

        objectsDetectedStruct[detPos][detectedObject].quat_x = obLoc.quat_x[detectedObject]; //assign object quaternion x to struct
        objectsDetectedStruct[detPos][detectedObject].quat_y = obLoc.quat_y[detectedObject]; //assign object quaternion y to struct
        objectsDetectedStruct[detPos][detectedObject].quat_z = obLoc.quat_z[detectedObject]; //assign object quaternion z to struct
        objectsDetectedStruct[detPos][detectedObject].quat_w = obLoc.quat_w[detectedObject]; //assign object quaternion w to struct

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

    int currentWeightingValue = trainingInfo.max_weighting / trainingInfo.times_trained; //calculate session weighting value
    if (DEBUG_detectedObjectCallback) {
        cout << "current weighting value is " << currentWeightingValue << endl;
    }

    if (totalObjectsDetectedStruct[detPos+1] == 0) {
        //no history to compare with, therefore do all the calculation stuff
        if (DEBUG_detectedObjectCallback) {
            cout << "nothing in struct pos " << detPos+1 << endl;
        }
        //first work out whether to add or delete from existing weighting value
        //no previous history, so add to weighting
        for (int detectedObject = 0; detectedObject < totalObjectsDetectedStruct[detPos]; detectedObject++) { //run through struct of pos 0
            //run through each detected object
            int getDetObjID = objectsDetectedStruct[detPos][detectedObject].id; //get id
            std::string getDetObjName = objectsDetectedStruct[detPos][detectedObject].object_name; //get name
            for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) { //run through entire context struct
                int getContextID = objectContext[isContext].object_id; //get context ID
                std::string getContextName = objectContext[isContext].object_name; //get context name
                if ((getDetObjID == getContextID) && (getDetObjName == getContextName)) { //if object ID and name are equal
                    //update object weighting and detected
                    int isCurrentWeighting = objectContext[isContext].object_weighting;
                    int isNewWeighting = isCurrentWeighting + trainingInfo.times_trained;
                    if (isNewWeighting > trainingInfo.max_weighting) { //if outside of max weighting
                        objectContext[isContext].object_weighting = trainingInfo.max_weighting;
                    }
                    else if (isNewWeighting < trainingInfo.min_weighting) { //if outside of min weighting
                        objectContext[isContext].object_weighting = trainingInfo.min_weighting;
                    }
                    else { //if inside bounding weight
                        objectContext[isContext].object_weighting = isNewWeighting;
                    }
                    //work out uniqueness
                }
            }
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
    ofstream FILE_WRITER;
	FILE_WRITER.open(context_info_loc);
    FILE_WRITER << trainingInfo.times_trained << "\n";
    FILE_WRITER.close();
    if (DEBUG_contextInfoToList) {
        cout << "finished saving context training information" << endl;
    }
}

/**
 * Last function to save all struct data into files, ready for using on next startup 
 */
void contextStructToList() {
    //object_id,object_name,object_confidence,object_detected,object_weighting,object_uniqueness,object_instances
    ofstream FILE_WRITER;
	FILE_WRITER.open(context_list_loc);
    for (int isObject = 0; isObject < totalObjectContextStruct; isObject++) {
        FILE_WRITER << 
        objectContext[isObject].object_id << "," << 
        objectContext[isObject].object_name << "," << 
        objectContext[isObject].object_confidence << "," << 
        objectContext[isObject].object_detected << "," << 
        objectContext[isObject].object_weighting << "," << 
        objectContext[isObject].object_uniqueness << "," << 
        objectContext[isObject].object_instances << "\n";
    }
    FILE_WRITER.close();
    if (DEBUG_contextStructToList) {
        cout << "finished saving context struct" << endl;
    }
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

    context_info_loc = wheelchair_dump_loc + dump_context_loc + context_info_name; //concatenate vars to create location of room list
    createFile(context_info_loc); //check to see if file is present, if not create a new one
    listToContextInfo(context_info_loc); //set context training info to struct


    ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 10, objectLocationsCallback); //full list of objects
    ros::Subscriber detected_objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 10, detectedObjectCallback); //detected objects in frame
    ros::Publisher object_context_pub = n.advertise<wheelchair_msgs::objectContext>("/wheelchair_robot/context/objects", 1000); //publish object context info for decision making
    ptr_object_context = &object_context_pub; //pointer to publish object context

    ros::Rate rate(10.0);
    while(ros::ok()) {
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    contextInfoToList(); //save training info
    contextStructToList(); //save object context
    return 0;
}