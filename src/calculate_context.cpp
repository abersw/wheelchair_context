#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>
//#include <getopt.h>
//#include <iostream> //io library
#include <fstream> //for writing and reading files in c++
#include <ros/package.h> //find ROS packages, needs roslib dependency

#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h"

#include <sstream>
using namespace std;

//variables and arrays for storing objects from training file
std::string softwareVersion = "Version 0.2 - Draft";

//contains list of rooms
struct Rooms {
	int id;
	std::string roomName;
	int timesTrained;
	int totalObjects;
};

//contains blueprint for training objects
struct Training {
	std::string objectName;
	double objectWeighting;
	int alreadyExists;
	double uniqueness;
};

struct ObjectDictionary {
	std::string objectName;
	int instances;
	int uniqueness;
};
//roomId followed by objects list
struct Training preTrained[1000][10000]; //saves items from file to struct
struct Training trained[1000][10000]; //struct for writing back to files
struct Rooms room[10000]; //list of rooms
struct ObjectDictionary objectDictionary[10000]; //objecs list to find uniqueness
struct ObjectDictionary preObjectDictionary[10000];

int totalRooms = 0;

//std::string roomNameROSParam; //won't need it

std::string objectsFileLoc; //variable for storing objects file location
std::string weightingFileLoc; //variable for storing the first section of weighting file location
std::string roomListLoc; //variable for storing room list location
std::string mobilenetFileType = ".objects"; //file extention for mobilenet type
std::string weightingFileType = ".weights"; //file extention for training types

//read room list
//struct all objects from weighting files
//update uniqueness of items
//weight found items against structed items
//output resulting room
//save uniqueness back to files

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

//calculate lines from files
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

void roomToStruct(std::string fileName) {
	std::string roomsDelimiter = ":";
	ifstream FILE_READER(fileName);
	std::string line;
	int roomNumber = 0;
	while (getline(FILE_READER, line)) {
		int delimiterPos = 0;
		std::string getRoomName; //temporary room name
		int getRoomId; //temporary room id
		getRoomName = line.substr(0, line.find(roomsDelimiter)); //string between 0 and delimiter
		room[roomNumber].roomName = getRoomName; //set room name
		cout << getRoomName; //print room name
		getRoomId = std::stoi(line.substr(line.find(roomsDelimiter) +1)); //get room id
		room[roomNumber].id = getRoomId; //set room id
		cout << getRoomId << "\n"; //print room id
		roomNumber++; //iterate to next room
	}
	FILE_READER.close();
}

void readTrainingFile(std::string fileName, int roomIdParam) {
	printSeparator(0);
	printf("DEBUG: readTrainingFile()\n");
	ofstream FILE_WRITER; //declare write file
	ifstream FILE_READER; //declare read file
	FILE_READER.open(fileName);
	if (FILE_READER.peek() == std::ifstream::traits_type::eof()) { //peek to see if file is empty
		cout << "weighting file is empty, starting to populate data. \n";
		FILE_READER.close();//closed for peeking
		FILE_WRITER.open(fileName); //open write file
		FILE_WRITER << room[roomIdParam].roomName << "\n";
		FILE_WRITER << 0; //first time training
		FILE_WRITER.close(); //close write file
		FILE_READER.open(fileName); //reopen file after peek
	}
	std::string line;
	int lineNumber = 0;
	int objectNumber = 0;
	while (getline(FILE_READER, line)) {
		if (lineNumber == 0) { //if line number is 0 - i.e. room name
			//do nothing room name
			cout << "reading Room Name: " << line << "\n";
		}
		else if (lineNumber == 1) { //if line number is 1 - i.e. training times
			//get times trained
			std::string getTimesTrainedString = line;
			int getTimesTrained = ::atoi(line.c_str()); //cast times trained string to int
			//getTimesTrained++; //not iterating for training
			room[roomIdParam].timesTrained = getTimesTrained; //set times trained to correponding room
			cout << "reading Times Trained: " << room[roomIdParam].timesTrained << "\n";
		}
		else if (lineNumber > 1) { //rest of the lines are trained objects
			//find delimiter positions
			std::string delimiter = ":"; //look for colon 
			int delimiterPos[5]; //set array of delimiter positions
			int delimiterNumber = 0; //current delimiter
			int lineLength = line.length(); //get length of line
			char lineArray[lineLength + 1]; //create array of chars
			strcpy(lineArray, line.c_str()); //set string to chars
			for (int charPos = 0; charPos < lineLength; charPos++) {
				if (lineArray[charPos] == ':') { //if char is colon
					//printf("%c\n", lineArray[i]);
					//printf("found delimiter\n");
					delimiterPos[delimiterNumber] = charPos; //add position of delimiter to array
					delimiterNumber++; //iterate to next delimiter
				}
			}


			//extract substrings between delimiters
			for (int section = 0; section < delimiterNumber +1; section++) { //go through line at each delimiter position
				if (section == 0) {
					preTrained[roomIdParam][objectNumber].objectName = line.substr(0, delimiterPos[0]); //set first substring to pretrained struct
					cout << "object number is " << objectNumber << "\n"; 
					cout << "preTrained objectname is: " + preTrained[roomIdParam][objectNumber].objectName + "\n";
				}
				else if (section == 1) {
					double weightingToDouble = std::atof(line.substr(delimiterPos[0] + 1, delimiterPos[1]).c_str()); //cast weighting from string to double
					preTrained[roomIdParam][objectNumber].objectWeighting = weightingToDouble; //set second substring to pretrained struct and cast to double
					cout << "preTrained objectWeighting is: " << preTrained[roomIdParam][objectNumber].objectWeighting << "\n";
				}
				else if (section == 2) {
					double uniquenessToDouble = std::atof(line.substr(delimiterPos[1] + 1).c_str()); //cast uniqueness from string to double
					preTrained[roomIdParam][objectNumber].uniqueness = uniquenessToDouble; //set third substring to pretrained struct and cast to double
					cout << "preTrained uniqueness is: " << preTrained[roomIdParam][objectNumber].uniqueness << "\n";
				}
			}
			delimiterNumber = 0; //set back to 0 when finished

			objectNumber+=1;
			//totalObjectsFromWeights = objectNumber;
			room[roomIdParam].totalObjects = objectNumber; //set number of objects for room struct
			cout << "total objects are " << room[roomIdParam].totalObjects << "\n";

		}
		lineNumber++;
	}
	FILE_READER.close();
	
	printSeparator(0);
}

void calculateUniqueness() {
	//notes for calculating uniqueness
	//uniqueness is percentage
	//if you have two rooms and it is located in 1, it's a unique item
	//occurances divided by total rooms
	// 1 / 2 = 0.5 * 100 = 50 - 50% common
	// 2 / 3 = 0.6667 * 100 = 66.66  |  100 - 66.6 = 33.34
	//what about rooms which haven't been trained as extensively? What affect does that have on uniqueness?

	//start with for loop of rooms
	int objectNumber = 0;
	int totalPreObjects = 0;
	int foundObject = 0;
	int foundObjectPos = 0;
	int tmpInstanceCounter = 0;
	for (int isRoom = 0; isRoom < totalRooms; isRoom++) { //iterate through each room
		for (int isObject = 0; isObject < room[isRoom].totalObjects; isObject++) { //iterate through each object from the room
			std::string getObjectName = preTrained[isRoom][isObject].objectName; //assign object name from room to variable
			for (int isPreObject = 0; isPreObject <= totalPreObjects; isPreObject++) { //iterate through temporary variable
				cout << "running through number " << isPreObject << "\n"; //debug line print iteration
				std::string getPreObjectName = preObjectDictionary[isPreObject].objectName; //assign object name from list
				cout << "pre object is " << getPreObjectName << "\n"; //debug line print object from list
				if (getObjectName == getPreObjectName) { //if object from room is the same as the list
					//tmpInstanceCounter++;
					foundObject = 1; //found object
					foundObjectPos = isPreObject; //
				}
				else {
					//found objects stay at 0
				}

			}//finish list loop
			if (foundObject == 1) { //if it found a match
				//preObjectDictionary[foundObjectPos].objectName = getObjectName;
				preObjectDictionary[foundObjectPos].instances += 1; //add 1 to instances found
				foundObject = 0; //set back to 0 for new iteration
			}
			else {
				//object not found, so add to array???
				preObjectDictionary[objectNumber].objectName = getObjectName; //create object in list
				preObjectDictionary[objectNumber].instances += 1; //add instance to object
				objectNumber++; //add 1 for next object
				totalPreObjects++; //increase size of objects to iterate
			}


			//go through whole list of objects
			//if u find a new one - add to array, detector activates when finished
			//if its the same add exists ++ (hopefully no new array)
		}
	}
	printSeparator(1);
	for (int i = 0; i < objectNumber; i++) {
		cout << preObjectDictionary[i].objectName << ":" << preObjectDictionary[i].instances << "\n";
		//cout << preObjectDictionary[i] << "\n";
	}
	//start working on adding instances back to preTrainedObjects array 
	//then calculate how unique they are, at the moment 1 
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "calculate_context");
  	ros::NodeHandle n;
  	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("calculate_context_topic", 1000);
  	printSeparator(0);
	printf("Calculate Context Software\n");
	printf("%s\n", softwareVersion.c_str());

  	printSeparator(0);
	std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
	//objectsFileLoc = wheelchair_dump_loc + "/dump/mobilenet/" + roomNameROSParam + mobilenetFileType;
	//weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/" + roomNameROSParam + weightingFileType;
	weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/";
	roomListLoc = wheelchair_dump_loc + "/dump/context_training/room.list";
	totalRooms = calculateLines(roomListLoc); //get number of rooms
	cout << totalRooms << "\n";
	roomToStruct(roomListLoc);

	//read training files
	for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
		std::string generateFileName = weightingFileLoc + room[isRoom].roomName + weightingFileType;
		readTrainingFile(generateFileName, isRoom);
	}

	/*for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
		std::string generateFileName = weightingFileLoc + room[isRoom].roomName + weightingFileType;
		calculateuniqueness(isRoom);
	}*/
	calculateUniqueness();


	return 0;
}