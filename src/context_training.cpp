/*
 * Tomos Fearn
 * tof7@aber.ac.uk
 * Context Training Software - Wheelchair Navigation
*/

/*
 * Todo:
 * ofstream instead of fopen
 * use associative arrays for room name, followed by struct of object name, confidence etc.
 * add rospack error for packages not found!!!
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
std::string roomListLoc;
std::string mobilenetFileType = ".objects";
std::string weightingFileType = ".weights";
//char objectsLocation[100]; //location of found objects file

//variables and arrays for storing objects from training file
std::string softwareVersion = "Version 0.1 - Draft";
std::string objectFileName = "../found-objects.txt";
//char* objectFileMode = "r"; //replace this later on with fstream



std::string roomNameROSParam;
int totalObjects = 0;
int totalRooms = 0;
//std::string objectsArray[10000]; //up to 10000 objects and two columns; name and confidence
struct Objects {
	std::string objectName;
	double objectConfidence;
};
struct Objects objects[10000];

struct Rooms {
	int id;
	std::string roomName;
};

struct Training {
	std::string objectName;
	double objectWeighting;
	int alreadyExists;
	double uniqueness;
};
struct Training preTrained[10000];
struct Training trained[1000][10000];
struct Rooms room[10000];
//struct Training preTrainedKitchen[10000];
//struct Training trainedKitchen[10000];
int timesTrained = 0;


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

void doesWheelchairDumpPkgExist() {
	if (ros::package::getPath("wheelchair_dump") == "") {
		cout << "FATAL:  Couldn't find package 'wheelchair_dump' \n";
		cout << "FATAL:  Closing training_context node. \n";
		printSeparator(1);
		ros::shutdown();
		exit(0);
	}
}

int createFile(std::string fileName) { //if this doesn't get called, no file is created
	printf("DEBUG: createFile()\n");
	std::ifstream fileExists(fileName);

	if (fileExists.good() == 1) {
		//File exists
		printf("Weighting file exists\n");
		//cout << fileName;
		return 1;
	}
	else {
		//File doesn't exist
		printf("Weighting file doesn't exist\n");
		printf("creating new file\n");

		ofstream NEW_FILE (fileName);
		NEW_FILE.close();
		//cout << fileName;
		return 0;
	}
}

int calculateLines(std::string fileName) {
	printf("DEBUG: calculateLines()\n");
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

//get files from mobilenet dump location and save to struct
void objectsFileToStruct(std::string fileName) {
	printf("DEBUG: objectsFileToStruct()\n");
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

//get list of rooms and save to struct
void roomListToStruct(std::string fileName) {
	printf("DEBUG: roomListToStruct()\n");
	std::string roomsDelimiter = ":";
	ifstream FILE_READER(fileName);
	std::string line;
	int roomNumber = 0;
	while (getline(FILE_READER, line)) {
		int delimiterPos = 0;
		std::string getRoomName;
		int getRoomId;
		getRoomName = line.substr(0, line.find(roomsDelimiter)); //string between 0 and delimiter
		room[roomNumber].roomName = getRoomName;
		cout << getRoomName;
		getRoomId = std::stoi(line.substr(line.find(roomsDelimiter) +1));
		room[roomNumber].id = getRoomId;
		cout << getRoomId << "\n";
		roomNumber++;
	}
}

void readTrainingFile(std::string fileName, int roomIdParam) {
	printSeparator(1);
	printf("DEBUG: readTrainingFile()\n");
	ifstream FILE_READER(fileName);
	std::string line;
	int lineNumber = 0;
	int objectNumber = 0;
	while (getline(FILE_READER, line)) {
		if (lineNumber == 0) {
			//do nothing room name
		}
		else if (lineNumber == 1) {
			//get times trained
			std::string getTimesTrained = line;
			timesTrained = ::atof(line.c_str());
			timesTrained++;
			//cout << timesTrained;
		}
		else if (lineNumber > 1) {
			//find delimiter positions
			std::string delimiter = ":";
			int delimiterPos[5];
			int delimiterNumber = 0;
			int lineLength = line.length();
			char lineArray[lineLength + 1];
			strcpy(lineArray, line.c_str()); 
			for (int charPos = 0; charPos < lineLength; charPos++) {
				if (lineArray[charPos] == ':') {
					//printf("%c\n", lineArray[i]);
					//printf("found delimiter\n");
					delimiterPos[delimiterNumber] = charPos;
					delimiterNumber++;
				}
			}


			//extract substrings between delimiters
			for (int section = 0; section < delimiterNumber +1; section++) {
				if (section == 0) {
					preTrained[lineNumber].objectName = line.substr(0, delimiterPos[0]);
					//cout << "preTrained objectname is: " + preTrained[lineNumber]->objectName + "\n";
				}
				else if (section == 1) {
					double weightingToDouble = std::atof(line.substr(delimiterPos[0] + 1, delimiterPos[1]).c_str()); 
					preTrained[lineNumber].objectWeighting = weightingToDouble;
					//cout << "preTrained objectWeighting is: " << preTrained[lineNumber]->objectWeighting << "\n";
				}
				else if (section == 2) {
					double uniquenessToDouble = std::atof(line.substr(delimiterPos[1] + 1).c_str());
					preTrained[lineNumber].uniqueness = uniquenessToDouble;
					//cout << "preTrained uniqueness is: " << preTrained[lineNumber]->uniqueness << "\n";
				}
			}
			delimiterNumber = 0; //set back to 0 when finished
			objectNumber++;
		}
		lineNumber++;
	}
	printSeparator(1);
}

void startTraining() {
	printf("DEBUG: startTraining()\n");
}







/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "context_training");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("context_training_topic", 1000);

  	printSeparator(0);
	printf("Training Context Software\n");
	printf("%s\n", softwareVersion.c_str());

	std::string roomNameROSParam;
	
	n.getParam("/wheelchair_robot/user/room_name", roomNameROSParam);
	if (!n.hasParam("/wheelchair_robot/user/room_name")) { //check program if room name param is not available
		std_msgs::String msg;

	    std::stringstream ss;
	    ss << "Couldn't get ROS PARAM room name";
	    msg.data = ss.str();

	    ROS_INFO("%s", msg.data.c_str());
		ros::shutdown();
		exit(0); //stop program if parameter room name is not available!
	}
	printf("Room name parameter is: %s\n", roomNameROSParam.c_str());

	doesWheelchairDumpPkgExist();


	std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
	objectsFileLoc = wheelchair_dump_loc + "/dump/mobilenet/" + roomNameROSParam + mobilenetFileType;
	weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/" + roomNameROSParam + weightingFileType;
	roomListLoc = wheelchair_dump_loc + "/dump/context_training/room.list";
	printf("%s\n", objectsFileLoc.c_str()); //print location of files
	printf("%s\n", weightingFileLoc.c_str());
	printf("%s\n", roomListLoc.c_str());
	printSeparator(1);


	/////////////////////////////////////////////////////////////////
	//get array of room names from file
	int roomListExists = createFile(roomListLoc); //create room list
	//add list of rooms to struct array
	roomListToStruct(roomListLoc);
	
	totalRooms = calculateLines(roomListLoc);
	printf("DEBUG: roomStruct\n");
	for (int i = 0; i < totalRooms; i++) {
		cout << room[i].roomName;
		cout << ":";
		cout << room[i].id;
		cout << "\n";
	}
	printSeparator(1);
	/////////////////////////////////////////////////////////////////




	createFile(weightingFileLoc); //creates new weighting file








	printSeparator(1);

	totalObjects = calculateLines(objectsFileLoc);
	//printf("total objects: %d\n", totalObjects);

	printSeparator(1);

	objectsFileToStruct(objectsFileLoc);

	printSeparator(1);
	printf("DEBUG: objectsStruct\n");
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

	//struct Training preTrainedKitchen[10000];
	readTrainingFile(weightingFileLoc, 0);
	printSeparator(1);
	//cout << preTrainedKitchen[0].objectName << ":" << preTrainedKitchen[0].objectWeighting << ":" << preTrainedKitchen[0].uniqueness << "\n";


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
