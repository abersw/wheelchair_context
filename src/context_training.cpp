/*
 * Tomos Fearn
 * tof7@aber.ac.uk
 * Context Training Software - Wheelchair Navigation
*/

/*
 * Todo:
 * ofstream instead of fopen
 * use associative arrays for room name, followed by struct of object name, confidence etc. -> not going to work - get list of rooms
 * struct Training preTrained[1000][10000]; first element is room id, second is list of objects





 * Parts finished
 * import mobilenet detected objects to struct
 * import room names and added new files back to file and struct
 * import weighting files - object name, confidence and uniqueness
*/

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
std::string softwareVersion = "Version 0.2 - Draft";



std::string roomNameROSParam;
int totalObjectsFromMnet = 0;
//int totalObjectsFromWeights = 0;
int totalRooms = 0;

int MIN_WEIGHTING = 0;
int MAX_WEIGHTING = 100;
int MAX_TRAINING_TIMES = 5;

//contains list of objects found by mobilenet
struct Objects {
	std::string objectName;
	double objectConfidence;
	int alreadyExists;
};
struct Objects objects[10000];

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
//roomId followed by objects list
struct Training preTrained[1000][10000]; //saves items from file to struct
struct Training currentlyTraining[1000][10000]; //calculating training storage
struct Training trained[1000][10000]; //struct for writing back to files
struct Rooms room[10000]; //list of rooms
//struct Training preTrainedKitchen[10000];
//struct Training trainedKitchen[10000];
//int timesTrained = 0;


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
	FILE_READER.close();
	totalObjectsFromMnet = objectNumber;
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
		room[roomNumber].roomName = getRoomName; //set room name
		cout << getRoomName; //print room name
		getRoomId = std::stoi(line.substr(line.find(roomsDelimiter) +1)); //get room id
		room[roomNumber].id = getRoomId; //set room id
		cout << getRoomId << "\n"; //print room id
		roomNumber++;
	}
	FILE_READER.close();
}

void restructRoomList() {
	cout << "DEBUG: restruct rooms list \n";
	//get array of room names from file
	int roomListExists = createFile(roomListLoc); //create room list
	//add list of rooms to struct array
	roomListToStruct(roomListLoc);

	totalRooms = calculateLines(roomListLoc);
	/*printf("DEBUG: roomStruct\n");
	for (int i = 0; i < totalRooms; i++) {
		cout << room[i].roomName;
		cout << ":";
		cout << room[i].id;
		cout << "\n";
	}*/

	//if room name doesn't exist in the struct, write back to the file, then add a weights file
	int foundRoomMatch = 0;
	for (int i = 0; i < totalRooms; i++) {
		if (room[i].roomName == roomNameROSParam) {
			//found a room match
			foundRoomMatch = 1;
		}
	}
	if (foundRoomMatch == 0) {
		//after loop, if a match hasn't been found - add room to file.
		ofstream WRITE_FILE(roomListLoc);
		for (int line = 0; line < totalRooms; line++) {
			WRITE_FILE << room[line].roomName << ":" << room[line].id << "\n";
		}
		WRITE_FILE << roomNameROSParam << ":" << totalRooms; //no need for return on last line of file
		WRITE_FILE.close();
		std::string createNewWeightingFile = weightingFileLoc + roomNameROSParam + weightingFileType;
		createFile(createNewWeightingFile);
	}
	totalRooms = calculateLines(roomListLoc);
	roomListToStruct(roomListLoc);
	printf("DEBUG: roomStruct\n");
	for (int i = 0; i < totalRooms; i++) {
		cout << room[i].roomName;
		cout << ":";
		cout << room[i].id;
		cout << "\n";
	}
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
		FILE_WRITER << roomNameROSParam << "\n";
		FILE_WRITER << 0; //first time training
		FILE_WRITER.close(); //close write file
		FILE_READER.open(fileName); //reopen file after peek
	}
	std::string line;
	int lineNumber = 0;
	int objectNumber = 0;
	while (getline(FILE_READER, line)) {
		if (lineNumber == 0) {
			//do nothing room name
			cout << "reading Room Name: " << line << "\n";
		}
		else if (lineNumber == 1) {
			//get times trained
			std::string getTimesTrainedString = line;
			int getTimesTrained = ::atoi(line.c_str());
			getTimesTrained++;
			room[roomIdParam].timesTrained = getTimesTrained;
			cout << "reading Times Trained: " << room[roomIdParam].timesTrained << "\n";
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
					preTrained[roomIdParam][objectNumber].objectName = line.substr(0, delimiterPos[0]);
					cout << "object number is " << objectNumber << "\n";
					cout << "preTrained objectname is: " + preTrained[roomIdParam][objectNumber].objectName + "\n";
				}
				else if (section == 1) {
					double weightingToDouble = std::atof(line.substr(delimiterPos[0] + 1, delimiterPos[1]).c_str());
					preTrained[roomIdParam][objectNumber].objectWeighting = weightingToDouble;
					cout << "preTrained objectWeighting is: " << preTrained[roomIdParam][objectNumber].objectWeighting << "\n";
				}
				else if (section == 2) {
					double uniquenessToDouble = std::atof(line.substr(delimiterPos[1] + 1).c_str());
					preTrained[roomIdParam][objectNumber].uniqueness = uniquenessToDouble;
					cout << "preTrained uniqueness is: " << preTrained[roomIdParam][objectNumber].uniqueness << "\n";
				}
			}
			delimiterNumber = 0; //set back to 0 when finished

			objectNumber+=1;
			//totalObjectsFromWeights = objectNumber;
			room[roomIdParam].totalObjects = objectNumber;
			cout << "total objects are " << room[roomIdParam].totalObjects << "\n";

		}
		lineNumber++;
	}
	FILE_READER.close();
	
	printSeparator(0);
}

void startTraining(std::string roomNameStringParam) { //training only runs one room
	printf("DEBUG: startTraining()\n");
	//this is currently the work in progress function
	
	int correspondingRoomId = 0;

	//run through for loop until room name parameter matches
	for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
		if (room[isRoom].roomName == roomNameStringParam) {
			cout << "found corresponding room name " << room[isRoom].roomName << "\n";
			correspondingRoomId = isRoom;
		}
		else {
			//do nothing, carry on looping
		}
	}
	std::string tmpon;
	std::string tmpwt;
	//iterate through objects list and find matches in weighting list
	for (int isMnetObject = 0; isMnetObject < totalObjectsFromMnet; isMnetObject++) {
		tmpon = objects[isMnetObject].objectName;
		for (int isWeightingObject = 0; isWeightingObject < room[correspondingRoomId].totalObjects; isWeightingObject++) {
			tmpwt = preTrained[correspondingRoomId][isWeightingObject].objectName;
			cout << tmpon << " and " << tmpwt << "\n";
		
			if ((preTrained[correspondingRoomId][isWeightingObject].alreadyExists == 1) || (objects[isMnetObject].alreadyExists == 1)) {
				//if it's already been set to 1 - then don't change back to 0 when not detected
				cout << "already matched \n";
			}
			else if ((preTrained[correspondingRoomId][isWeightingObject].alreadyExists == 0) && (objects[isMnetObject].alreadyExists == 0)) {
				if (objects[isMnetObject].objectName == preTrained[correspondingRoomId][isWeightingObject].objectName) {
					//found first match
					cout << "found match \n";
					preTrained[correspondingRoomId][isWeightingObject].alreadyExists = 1;
					objects[isMnetObject].alreadyExists = 1;
				}
				else {
					//didn't find match
					cout << "didn't find match: " << preTrained[correspondingRoomId][isWeightingObject].objectName << " : " << objects[isMnetObject].objectName << "\n";

					preTrained[correspondingRoomId][isWeightingObject].alreadyExists = 0;
					objects[isMnetObject].alreadyExists = 0;
				}
			}
		}
	}



	//print debug info
	printSeparator(0);
	cout << "DEBUG existing weights \n";
	cout << "total objects from weights " << room[0].totalObjects << "\n";
	for (int i = 0; i < room[0].totalObjects; i++) {
		cout << preTrained[0][i].objectName << " : " << preTrained[0][i].alreadyExists << "\n";
	}
	printSeparator(0);
	cout << "DEBUG existing objects \n";
	cout << "total objects from mnet " << totalObjectsFromMnet << "\n";

	for (int i = 0; i < totalObjectsFromMnet; i++) {
		cout << objects[i].objectName << " : " << objects[i].alreadyExists << "\n";
	}

	printSeparator(0);
	int currentTrainingPos = 0;
	int weightingValueCalc = 0; //variable stores divider for object weighting
	//move items into currentTraining (pre calculated, but organised)
	if (room[correspondingRoomId].timesTrained <= MAX_TRAINING_TIMES) {
		double weightingDividerValue = room[correspondingRoomId].timesTrained;
		weightingValueCalc = MAX_WEIGHTING / weightingDividerValue;
	}
	else if (room[correspondingRoomId].timesTrained > MAX_TRAINING_TIMES) {
		double weightingDividerValue = MAX_TRAINING_TIMES;
		weightingValueCalc = MAX_WEIGHTING / weightingDividerValue;
	}

	for (int isWeightingObject = 0; isWeightingObject < room[correspondingRoomId].totalObjects; isWeightingObject++) {
		if (preTrained[correspondingRoomId][isWeightingObject].alreadyExists == 1) { //if one exists - add to weighting
			trained[correspondingRoomId][currentTrainingPos].objectName = preTrained[correspondingRoomId][isWeightingObject].objectName; //get and set object name
			int isCurrentWeighting = preTrained[correspondingRoomId][isWeightingObject].objectWeighting; //get current weighting value
			int newWeightingValue = isCurrentWeighting + weightingValueCalc;
			cout << isCurrentWeighting << " is current weighting \n";
			cout << weightingValueCalc << " is divider\n";
			cout << newWeightingValue << " is new weighting value\n";
			if (newWeightingValue > MAX_WEIGHTING) {
				trained[correspondingRoomId][currentTrainingPos].objectWeighting = MAX_WEIGHTING; //set to max weighting if higher
				cout << trained[correspondingRoomId][currentTrainingPos].objectWeighting << " is set weighting\n";
			}
			else {
				trained[correspondingRoomId][currentTrainingPos].objectWeighting = newWeightingValue; //set to new weighting value
				cout << trained[correspondingRoomId][currentTrainingPos].objectWeighting << " is set weighting\n";
			}
			trained[correspondingRoomId][currentTrainingPos].uniqueness = preTrained[correspondingRoomId][isWeightingObject].uniqueness; //get and set uniqueness
		}
		else if (preTrained[correspondingRoomId][isWeightingObject].alreadyExists == 0) {//if it doesn't exist - subtract weighting
			trained[correspondingRoomId][currentTrainingPos].objectName = preTrained[correspondingRoomId][isWeightingObject].objectName; //get and set object name
			int isCurrentWeighting = preTrained[correspondingRoomId][isWeightingObject].objectWeighting; //get current weighting value
			int newWeightingValue = isCurrentWeighting - weightingValueCalc;
			cout << isCurrentWeighting << " is current weighting \n";
			cout << weightingValueCalc << " is divider\n";
			cout << newWeightingValue << " is new weighting value\n";
			if (newWeightingValue < MIN_WEIGHTING) {
				trained[correspondingRoomId][currentTrainingPos].objectWeighting = MIN_WEIGHTING; //set to min weighting if lower
				cout << trained[correspondingRoomId][currentTrainingPos].objectWeighting << " is set weighting\n";
			}
			else {
				trained[correspondingRoomId][currentTrainingPos].objectWeighting = newWeightingValue; //set to new weighting value
				cout << trained[correspondingRoomId][currentTrainingPos].objectWeighting << " is set weighting\n";
			}
			trained[correspondingRoomId][currentTrainingPos].uniqueness = preTrained[correspondingRoomId][isWeightingObject].uniqueness; //get and set uniqueness
		}
		currentTrainingPos++;
	}

	for (int isMnetObject = 0; isMnetObject < totalObjectsFromMnet; isMnetObject++) {
		if (objects[isMnetObject].alreadyExists == 1) {
			//do nothing because it has already been added
		}
		else if (objects[isMnetObject].alreadyExists == 0) {
			trained[correspondingRoomId][currentTrainingPos].objectName = objects[isMnetObject].objectName; //get and set object name
			int isCurrentWeighting = 0; //get current weighting value of 0
			int newWeightingValue = isCurrentWeighting + weightingValueCalc;
			cout << isCurrentWeighting << " OB: is current weighting \n";
			cout << weightingValueCalc << " OB: is divider\n";
			cout << newWeightingValue << " OB: is new weighting value\n";
			if (newWeightingValue > MAX_WEIGHTING) {
				trained[correspondingRoomId][currentTrainingPos].objectWeighting = MAX_WEIGHTING; //set to max weighting if higher
				cout << trained[correspondingRoomId][currentTrainingPos].objectWeighting << " OB: is set weighting\n";
			}
			else {
				trained[correspondingRoomId][currentTrainingPos].objectWeighting = newWeightingValue; //set to new weighting value
				cout << trained[correspondingRoomId][currentTrainingPos].objectWeighting << " OB: is set weighting\n";
			}
			trained[correspondingRoomId][currentTrainingPos].uniqueness = 0; //set uniqueness as 0
			currentTrainingPos++; //only iterate for new objects
		}
	}
	int totalTrained = currentTrainingPos;
	printSeparator(0);


	for (int i = 0; i < totalTrained; i++) {
		cout << trained[correspondingRoomId][i].objectName << trained[correspondingRoomId][i].objectWeighting << trained[correspondingRoomId][i].uniqueness << "\n";
	}

	//for object in mnet not matched, add to current training - is present add to probability

	//


/*
	//the following nested statement sets already exists flag when it finds matches in pretrained weights file and mnet objects file
	for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
		//get room id
		cout << "total rooms is " << totalRooms << "\n";
		cout << "roomID is: " << room[isRoom].id << room[isRoom].roomName << "\n";
		cout << "total objects from weights " << totalObjectsFromWeights << "\n";
		for (int isMnetObject = 0; isMnetObject < totalObjectsFromMnet; isMnetObject++) { //detected objects will be present, weightings may not be
			//iterate through pretrained struct from weighting file
			//cout << "weighting object is " << preTrained[isRoom][isWeightingObject].objectName << "\n";
			for (int isWeightingObject = 0; isWeightingObject < totalObjectsFromWeights; isWeightingObject++) {
				//iterate through each object found by Mobilenet
				//look for matching pairs
				if ((objects[isMnetObject].objectName == preTrained[isRoom][isWeightingObject].objectName) && 
					(preTrained[isRoom][isWeightingObject].alreadyExists != 1)) { //look for matching objects in weighting file and check they haven't already been flagged as existing
					//cout << "Found matching object names \n";
					//add objects to
					//set flag to existing
					preTrained[isRoom][isWeightingObject].alreadyExists = 1; //set object matches to already exists
					objects[isMnetObject].isNew = 0;
					//print out the matching pairs
					cout << "set " << preTrained[isRoom][isWeightingObject].objectName << " and " << objects[isMnetObject].objectName << " as match \n";
				}
				else if ((objects[isMnetObject].objectName == preTrained[isRoom][isWeightingObject].objectName) && 
					(preTrained[isRoom][isWeightingObject].alreadyExists == 1)) { 
					//don't do anything if it has already been matched
				}
				else {
					objects[isMnetObject].isNew
				}
			}
		}
	}
	*/

	//try two
	/*for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
		for ()
	}*/

	//the following function adds the pretrained data and merges objects struct (excluding matching objects)
	/*for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
		//loop through every room
		for (int isWeightingObject = 0; isWeightingObject < totalObjectsFromWeights; isWeightingObject++) {
			//loop through pretrained objects

		}
	}*/

	/*for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
		for (int isWeightingObject = 0; isWeightingObject < totalObjectsFromWeights; isWeightingObject++) {
			for (int isMnetObject = 0; isMnetObject < totalObjectsFromMnet; isMnetObject++) {
				//do stuff
				//currentlyTraining[isRoom][isWeightingObject].roomName = preTrained.
			}
		}
	}*/
}

void structToWeightingFile() {

}





/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	//set up ROS node, get all parameters etc
  	ros::init(argc, argv, "context_training");
  	ros::NodeHandle n;
  	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("context_training_topic", 1000);

  	printSeparator(0);
	printf("Training Context Software\n");
	printf("%s\n", softwareVersion.c_str());

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
	//weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/" + roomNameROSParam + weightingFileType;
	weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/";
	roomListLoc = wheelchair_dump_loc + "/dump/context_training/room.list";
	printf("%s\n", objectsFileLoc.c_str()); //print location of files
	printf("%s\n", weightingFileLoc.c_str());
	printf("%s\n", roomListLoc.c_str());
	printSeparator(1);


	/////////////////////////////////////////////////////////////////

	restructRoomList();

	printSeparator(1);
	/////////////////////////////////////////////////////////////////




	//createFile(weightingFileLoc); //creates new weighting file - not needed any more <- moved to restruct








	printSeparator(1);

	totalObjectsFromMnet = calculateLines(objectsFileLoc);
	//printf("total objects: %d\n", totalObjectsFromMnet);

	printSeparator(1);

	objectsFileToStruct(objectsFileLoc);

	printSeparator(1);
	printf("DEBUG: objectsStruct\n");
	for (int i = 0; i < totalObjectsFromMnet; i++) {
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




	/////////////////////////////////////////////////////////////////

	//struct Training preTrainedKitchen[10000];
	//populate 2d array of [room][objects] -> pass this to readtrainingfile as parameter
	for (int i = 0; i < totalRooms; i++) {
		//get roomname from corresponding position in for loop, add file extention and pass to function
		std::string generateRoomWeightFile = weightingFileLoc + room[i].roomName + weightingFileType;
		readTrainingFile(generateRoomWeightFile, i); //2nd param is room id
	}
	printSeparator(1);
	//cout << preTrainedKitchen[0].objectName << ":" << preTrainedKitchen[0].objectWeighting << ":" << preTrainedKitchen[0].uniqueness << "\n";
	/*for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			cout << "pretrained debug " << preTrained[i][j].objectName << "\n";
		}
	}*/

	/////////////////////////////////////////////////////////////////
	startTraining(roomNameROSParam);

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
			cout << "Finished Training \n";
			ros::shutdown();
	    //loop_rate.sleep();
	    ++count;
	    doOnce = 0;
	}
  }


  return 0;
}
