/*
 * Tomos Fearn
 * tof7@aber.ac.uk
 * Context Training Software - Wheelchair Navigation
*/

/*
 * Todo:
 * ofstream instead of fopen
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <iostream> //io library
#include <fstream> //for writing and reading files in c++
#include <ros/package.h> //find ROS packages, needs roslib dependency

#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h"

#include <sstream>
using namespace std;


//from old training file
#define MAX_STRING_SIZE 40

FILE *filePointer; //pointer for file reader/writer

std::string objectsFileLoc;
std::string weightingFileLoc;
std::string mobilenetFileType = ".objects";
std::string weightingFileType = ".weights";
//char objectsLocation[100]; //location of found objects file

//variables and arrays for storing objects from training file
std::string softwareVersion = "Version 0.1 - Draft";
std::string objectFileName = "../found-objects.txt";
//char* objectFileMode = "r"; //replace this later on with fstream




std::string roomName;
int totalObjects = 0;
//std::string objectsArray[10000]; //up to 10000 objects and two columns; name and confidence
struct Objects {
	std::string objectName;
	double objectConfidence;
};
struct Objects objects[10000];


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

int isFirstTimeTraining(std::string fileName) { //if this doesn't get called, no file is created
	std::ifstream fileExists(fileName);

	if (fileExists.good() == 1) {
		//File exists
		printf("Weighting file exists\n");
		cout << fileName;
		return 1;
	}
	else {
		//File doesn't exist
		printf("Weighting file doesn't exist\n");
		printf("creating new file\n");

		ofstream NEW_FILE (fileName);
		NEW_FILE.close();
		cout << fileName;
		return 0;
	}
}

int calculateLines(std::string fileName) {
	ifstream FILE_COUNTER(fileName);
	std::string getlines;
	int returnCounter = 0;
	while (getline (FILE_COUNTER, getlines)) {
		returnCounter++;
  		// Output the text from the file
  		//cout << getlines;
  		//cout << "\n";
	}
	FILE_COUNTER.close();
	return returnCounter;
}

void objectsFileToStruct(std::string fileName) {
	std::string objectsDelimiter = ":";
	ifstream FILE_READER(fileName);
	std::string line;
	int objectNumber = 0;
	while (getline(FILE_READER, line)) {
		int delimiterPos = 0;
		std::string getObjectName;
		std::string getObjectConfidence;
		getObjectName = line.substr(0, line.find(objectsDelimiter)); //string between pos 0 and delimiter
		//cout << getObjectName; //print object name
		getObjectConfidence = line.substr(line.find(objectsDelimiter) +1); //string between delimiter and end of line
		objects[objectNumber].objectName = getObjectName;
		//cout << ::atof(getObjectConfidence.c_str()); //print object confidence
		double getObjectConfidence2Double = std::atof(getObjectConfidence.c_str());
		objects[objectNumber].objectConfidence = getObjectConfidence2Double;
		objectNumber++;
	}
}







/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "context_training");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("context_training_topic", 1000);

	printf("Training Context Software\n");
	printf("%s\n", softwareVersion.c_str());

	std::string roomNameROSParam;
	
	n.getParam("/wheelchair_robot/user/room_name", roomNameROSParam);
	printf("Room name parameter is: %s\n", roomNameROSParam.c_str());
	roomName = roomNameROSParam; //set to global variable

	//std::string path = ros::package::getPath("roslib");
	std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
	objectsFileLoc = wheelchair_dump_loc + "/dump/mobilenet/" + roomName + mobilenetFileType;
	weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/" + roomName + weightingFileType;
	printf("%s\n", objectsFileLoc.c_str()); //print location of files
	printf("%s\n", weightingFileLoc.c_str());
	printSeparator(1);

	int firstTimeTraining = isFirstTimeTraining(weightingFileLoc); //creates new weighting file

	printSeparator(1);

	totalObjects = calculateLines(objectsFileLoc);
	printf("total objects: %d\n", totalObjects);

	printSeparator(1);

	objectsFileToStruct(objectsFileLoc);

	printSeparator(1);
	for (int i = 0; i < totalObjects; i++) {
		cout << objects[i].objectName;
		cout << ":";
		cout << objects[i].objectConfidence;
		cout << "\n";
	}
/*
	ifstream MyReadFile(objectsLocation);
	std::string getlines;
	while (getline (MyReadFile, getlines)) {

  		// Output the text from the file
  		cout << getlines;
  		cout << "\n";
	}
	MyReadFile.close();
*/




  ros::Rate loop_rate(10);
  int doOnce = 1;

  int count = 0;
  while (ros::ok())
  {
  	if (doOnce == 1) {
	    /**
	     * This is a message object. You stuff it with data, and then publish it.
	     */
	    std_msgs::String msg;

	    std::stringstream ss;
	    ss << "hello world " << count;
	    msg.data = ss.str();

	    ROS_INFO("%s", msg.data.c_str());

	    /**
	     * The publish() function is how you send messages. The parameter
	     * is the message object. The type of this object must agree with the type
	     * given as a template parameter to the advertise<>() call, as was done
	     * in the constructor above.
	     */
	    chatter_pub.publish(msg);

	    ros::spinOnce();
			printf("spinned once\n");
			ros::shutdown();
	    //loop_rate.sleep();
	    ++count;
	    doOnce = 0;
	}
  }


  return 0;
}
