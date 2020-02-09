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

std::string objectsLocation;
std::string weightingFile;
std::string mobilenetFileType = ".objects";
std::string weightingFileType = ".weights";
//char objectsLocation[100]; //location of found objects file

//variables and arrays for storing objects from training file
std::string softwareVersion = "Version 0.1 - Draft";
std::string objectFileName = "../found-objects.txt";
//char* objectFileMode = "r"; //replace this later on with fstream




std::string roomName;

/*
int totalCharacters = 0;
int countObjectLines;
char objectList[10000][MAX_STRING_SIZE]; //array for storing object in each element followed by characters
char objectCharArray[10000]; //array for storing individual characters

int countTrainingLines;
char trainingList[10000][MAX_STRING_SIZE];
char trainingCharArray[10000];

struct Training {
	char objectName[1][MAX_STRING_SIZE];
	int objectWeighting;
	int alreadyExists;
};

struct Training objectsStuct[1000];
struct Training preTrained[1000];
struct Training trained[1000];
int MAX_TRAIN_VALUE = 100;
int MIN_TRAIN_VALUE = 0;
int MAX_TRAINING_TIMES = 5;
int sizeOfTrained = 0;

char preTrainedObjects[1000][MAX_STRING_SIZE];
int preTrainedValues[1000];

char postTrainedObjects[1000][MAX_STRING_SIZE];
int postTrainedValues[1000];
int preTrainedLines = 0;
char saveBuffer[10000][MAX_STRING_SIZE];

//variables and arrays for weighting file
int countWeightingLines;
int totalTrainingCharacters;

char roomName[40];
int timesTrained = 0;
*/

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
	objectsLocation = wheelchair_dump_loc + "/dump/mobilenet/" + roomName + mobilenetFileType;
	weightingFile = wheelchair_dump_loc + "/dump/context_training/" + roomName + weightingFileType;
	printf("%s\n", objectsLocation.c_str()); //print location of files
	printf("%s\n", weightingFile.c_str());
	printSeparator(1);

	int firstTimeTraining = isFirstTimeTraining(weightingFile); //creates new weighting file

	printSeparator(1);
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
	    //loop_rate.sleep();
	    ++count;
	    doOnce = 0;
	}
  }


  return 0;
}
