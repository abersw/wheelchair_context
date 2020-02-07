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
#include <fstream> //for writing and reading files
#include <ros/package.h> //find ROS packages, needs roslib dependency

#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h"

#include <sstream>
using namespace std;


//from old training file
#define MAX_STRING_SIZE 40

FILE *filePointer; //pointer for file reader/writer

std::string objectsLocation;
std::string mobilenetFileType ".objects"
//char objectsLocation[100]; //location of found objects file

//variables and arrays for storing objects from training file
string softwareVersion = "Version 0.1 - Draft";
string objectFileName = "../found-objects.txt";
char* objectFileMode = "r";


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

long calculateLines(FILE *filePointer, char fileName[], char fileMode[]) {
    long countLines = 0;
    filePointer = fopen(fileName, fileMode);

    for (char c = getc((FILE*)filePointer); c != EOF; c = getc((FILE*)filePointer)) {
        if (c == '\n') { // Increment count if this character is newline
            countLines++;
        }
    }
    //printf("Number of lines: %ld\n", countLines);
    fclose((FILE*)filePointer);
    return countLines;
}

void objectFileToArray(FILE *filePointer, char fileName[], char fileMode[]) {
	int itemNumber = 0;
	int charNumber = 0;
	filePointer = fopen(fileName, fileMode); //open file
	for (int item = 0; item < countObjectLines; item++) {
		for (char currentChar = getc((FILE*)filePointer); currentChar != '\n'; currentChar = getc((FILE*)filePointer)) {
			//printf("%c", currentChar);
			objectCharArray[charNumber] = currentChar;
			charNumber++;
		}
		objectCharArray[charNumber] = '\n';
		charNumber++; //move along one character for next object (word)
	}
	fclose((FILE*)filePointer);
	totalCharacters = charNumber;
}

void trainingFileToArray(FILE *filePointer, char fileName[], char fileMode[]) {
	int itemNumber = 0;
	int charNumber = 0;
	filePointer = fopen(fileName, fileMode); //open file
	for (int item = 0; item < countWeightingLines; item++) {
		for (char currentChar = getc((FILE*)filePointer); currentChar != '\n'; currentChar = getc((FILE*)filePointer)) {
			//printf("%c", currentChar);
			trainingCharArray[charNumber] = currentChar;
			charNumber++;
		}
		trainingCharArray[charNumber] = '\n';
		charNumber++; //move along one character for next object (word)
	}
	fclose((FILE*)filePointer);
	totalTrainingCharacters = charNumber;
}

void objectArrayToString() { //convert char array to multidimensional array
	int wordCount = 0;
	int characterCount = 0;
	for (int i = 0; i <= totalCharacters; i++) {
		if (objectCharArray[i] != '\n') {
			objectList[wordCount][characterCount] = objectCharArray[i];
			characterCount++;
		}
		else {
			wordCount++;
			characterCount = 0;
		}
	}
}


void trainingArrayToString() {
	int wordCount = 0; //start at 2 to avoid writing over header
	int characterCount = 0;

	for (int i = 0; i <= totalTrainingCharacters; i++) {
		if (trainingCharArray[i] != '\n') {
			trainingList[wordCount][characterCount] = trainingCharArray[i];
			characterCount++;
		}
		else {
			wordCount++;
			characterCount = 0;
		}
	}
}

int trainingFileExists(char * fileName) {
	if (access(fileName, F_OK) != -1) {
		//File exists
		printf("training file exists\n");
		return 1;
	}
	else {
		//File doesn't exist
		printf("ERROR: training file doesn't exist\n");
		exit(0); //stop program if training file doesn't exist
	}
}

int isFirstTimeTraining(char * roomName, char * fileName) { //if this doesn't get called, no file is created
	if (access(fileName, F_OK) != -1) {
		//File exists
		printf("Weighting file exists\n");
		return 1;
	}
	else {
		//File doesn't exist
		printf("Weighting file doesn't exist\n");
		printf("creating new file\n");

		FILE *newFile = fopen(fileName, "w");
		fprintf(newFile, "%s\n", roomName); //add room name to header
		fprintf(newFile, "%d\n", 0); //add number of training sessions to header
		fclose(newFile);
		return 0;
		//exit(0); //stop program
	}
}


void trainingListToStruct() {
	strcat(roomName, trainingList[0]); //convert char array to correct variable
	timesTrained = atoi(trainingList[1]); //convert char array to int variable
	int structId = 0;
	printf("roomName is %s\n", roomName);
	printf("times trained is %d\n", timesTrained);
	char buffer1[1][40];
	char buffer2[1][40];
	int iteratePreTrainedPos = 0;

	for (int i = 2; i <countWeightingLines; i++) { //line number starting infront of the header
		int lineLength = strlen(trainingList[i]); //calculate string length of line
		int foundFlag = 0;
		int counter = 0;
		int newDelimiterPos = 0;
		int oldDelimiterPos = 0;
		memset(&buffer1, 0, sizeof(buffer1));
		memset(&buffer2, 0, sizeof(buffer2));
		for (int j = 0; j < lineLength; j++) { //character number
			//printf("stct: %c\n", trainingList[i][j]); //print off each character from trainingList
			if ((foundFlag == 0) && (trainingList[i][j] != ':')) { //not found first delimiter
				buffer1[0][counter] = trainingList[i][j];
				counter++;
			}
			else if ((foundFlag == 0) && (trainingList[i][j] == ':')) { //found first delimiter
				printf("found delimiter on line %d position %d\n", i, j); //identify location of delimiter
				newDelimiterPos = j;
				foundFlag = 1;
				counter = 0;

			}
			else if ((foundFlag == 1) && (trainingList[i][j] != ':')) {
				buffer2[0][counter] = trainingList[i][j];
				counter++;
			}
		}

		//printf("found end of line\n");
		//printf("predelimiter is %s\n", buffer1[0]);
		int valueStringToInt = atoi(buffer2[0]);
		//printf("postdelimiter is %d\n", valueStringToInt);

		//strcat(preTrainedObjects[iteratePreTrainedPos], buffer1[0]);
		strcat(preTrained[iteratePreTrainedPos].objectName[0], buffer1[0]);
		printf("struct objectName is %s\n", preTrained[iteratePreTrainedPos].objectName[0]);
		preTrained[iteratePreTrainedPos].objectWeighting = valueStringToInt;
		printf("struct objectWeighting is %d\n", preTrained[iteratePreTrainedPos].objectWeighting);
		iteratePreTrainedPos++;
		//preTrainedObjects[iteratePreTrainedPos] = buffer1[0];

		//strcat(preTrained.objectName)
		//printf("eof\n");
	}
	preTrainedLines = iteratePreTrainedPos;
	/*printf("weighting lines is %d\n", countWeightingLines);
	for (int j = 0; j < countWeightingLines; j++) { //starting at 2 will skip the header data
		//printf("%d\n", countLines);
		//int objectStringLength = strlen(objectList[i]);
		printf("training list is %s", trainingList[j]); //this is now apparently a string!
	}
	printf("testing some bugs\n");
	printf("%s", trainingList[0]);
	printf("%s", trainingList[1]);
	printf("%s", trainingList[2]);*/
}


void calculateWeightings() {
	printf("weighting lines %d\n", preTrainedLines);
	/*for (int i = 0; i < preTrainedLines; i++) {
		for (int j = 0; j < countObjectLines; j++) {
			printf("%s", preTrained[i].objectName[0]);
			printf("%s", objectList[j]);
			int result = strcmp(preTrained[i].objectName[0], objectList[j]);
			//printf("%d\n", result);
			//flag matches first
			if (result == 0) { //found a matching string
				printf("found match\n");
				preTrained[i].alreadyExists = 1;
			}
			else {
				if (preTrained[i].alreadyExists == 1) {
					//ignore
				}
				else {
					printf("didn't find match\n");
					preTrained[i].alreadyExists = 0;
				}

			}
			//if count object lines is less than or equal trained lines

			//if two arrays are same length - it must be equal
		}
	}*/
	for (int i = 0; i < preTrainedLines; i++) {
		for (int j = 0; j < countObjectLines; j++) {
			int result = strcmp(preTrained[i].objectName[0], objectsStuct[j].objectName[0]);
			printf("%s", preTrained[i].objectName[0]);
			printf("%s\n", objectsStuct[j].objectName[0]);

			if ((preTrained[i].alreadyExists == 1) || (objectsStuct[j].alreadyExists == 1)) {
				printf("already found a match\n");
			}
			else if ((preTrained[i].alreadyExists == 0) && (objectsStuct[j].alreadyExists == 0)) {
				if (result == 0) {
					preTrained[i].alreadyExists = 1;
					objectsStuct[j].alreadyExists = 1;
					printf("found first match\n");

				}
				else {
					preTrained[i].alreadyExists = 0;
					objectsStuct[j].alreadyExists = 0;
					printf("didn't find match\n");
				}
			}
			else {
				printf("ERROR: inconsistency\n");
			}
		}
	}
	for (int i = 0; i < countObjectLines; i++) { //test to make sure all the objects appear as found
		printf("existing %s\n", objectsStuct[i].objectName[0]);
		printf("existing %d\n", objectsStuct[i].alreadyExists);
	}

	for (int i = 0; i < countObjectLines; i++) {
		printf("object struct is %s\n", objectsStuct[i].objectName[0]);
		for (int j = 0; j < preTrainedLines; j++) {
			//int result = strcmp(objectsStuct[i].objectName[0], )
		}
	}
	//struct now contains flags of objects that already exist

	//find already existing items and perform calculation based on how many times they have been trained

	if (timesTrained < MAX_TRAINING_TIMES) {
		timesTrained++;
	}
	else {
		//don't do anything
	}

	int trainedCounter = 0;
	for (int k = 0; k < preTrainedLines; k++) {
		int weightValue = (MAX_TRAIN_VALUE / timesTrained);
		strcat(trained[trainedCounter].objectName[0], preTrained[k].objectName[0]);
		if (preTrained[k].alreadyExists == 1) { //it will add if it exists
			int newWeighting = preTrained[k].objectWeighting + weightValue;
			if (newWeighting > MAX_TRAIN_VALUE) {
				trained[trainedCounter].objectWeighting = MAX_TRAIN_VALUE;
				trainedCounter++;
			}
			else {
				trained[trainedCounter].objectWeighting = newWeighting;
				trainedCounter++;
			}
		}
		else if (preTrained[k].alreadyExists == 0) {
			int newWeighting = preTrained[k].objectWeighting - weightValue;
			if (newWeighting < MIN_TRAIN_VALUE) {
				trained[trainedCounter].objectWeighting = 0;
				trainedCounter++;
			}
			else {
				trained[trainedCounter].objectWeighting = newWeighting;
				trainedCounter++;
			}
		}
	}
	//find new items and perform the default 100 calculation
	for (int l = 0; l < countObjectLines; l++) {
		if (objectsStuct[l].alreadyExists == 0) { //new objects found
			strcat(trained[trainedCounter].objectName[0], objectsStuct[l].objectName[0]);
			trained[trainedCounter].objectWeighting = MAX_TRAIN_VALUE;
			trainedCounter++;
		}
		else {
			//don't do anything
		}
	}
	sizeOfTrained = trainedCounter;

	//add to trained struct

	//save trained struct back to file
}

void startTraining(FILE *filePointer, char fileName[], char fileMode[]) {
	countWeightingLines = calculateLines((FILE*)filePointer, fileName, objectFileMode);
	//printf("number of training lines are %d\n", countWeightingLines);
	//getWeightingHeader((FILE*)filePointer, fileName, objectFileMode); //set roomName and timesTrained
	//readWeightingFile((FILE*)filePointer, fileName, objectFileMode);
	if (timesTrained == 0) {
		printf("weighting file is empty, start initial object adding\n");

		//initialObjectTraining((FILE*)filePointer, fileName, objectFileMode);
	}
	else {
		printf("weighting file is not empty, start reading file\n");
	}
	trainingFileToArray((FILE*)filePointer, fileName, objectFileMode);
	trainingArrayToString();
	trainingListToStruct();



	//filePointer = fopen(fileName, fileMode);
	//readWeightingFile((FILE*)filePointer, fileName, fileMode);
}

void saveToTrainingFile(FILE *filePointer, char fileName[], char fileMode[]) {
	filePointer = fopen(fileName, fileMode); //open file
	char buffer1[1][MAX_STRING_SIZE];
	char buffer2[1][MAX_STRING_SIZE];
	memset(&buffer1, 0, sizeof(buffer1));
	memset(&buffer2, 0, sizeof(buffer2));
	strcat(buffer1[0], roomName);
	strcat(buffer1[0], "\n");
	printf("%s", buffer1[0]);
	fprintf(filePointer, "%s", buffer1[0]);

	char timesTrainedToChar[10];
	sprintf(timesTrainedToChar, "%d", timesTrained);
	strcat(buffer2[0], timesTrainedToChar);
	strcat(buffer2[0], "\n");
	printf("%s", buffer2[0]);
	fprintf(filePointer, "%s", buffer2[0]);

	for (int i = 0; i < sizeOfTrained; i++) {
		char buffer3[1][40];
		memset(&buffer3, 0, sizeof(buffer3));
		strcat(buffer3[0], trained[i].objectName[0]);
		strcat(buffer3[0], ":");
		char weightingToChar[10];
		sprintf(weightingToChar, "%d", trained[i].objectWeighting);
		strcat(buffer3[0], weightingToChar);
		strcat(buffer3[0], "\n");
		printf("%s", buffer3[0]);
		fprintf(filePointer, "%s", buffer3[0]);
	}
	fclose((FILE*)filePointer);
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
	n.getParam("/param/context_training/room_name", roomNameROSParam);
	printf("Room name parameter is: %s\n", roomNameROSParam.c_str());

	//std::string path = ros::package::getPath("roslib");
	std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
	objectsLocation = wheelchair_dump_loc + "/dump/mobilenet/" + roomNameROSParam + mobilenetFileType;
	printf("%s\n", objectsLocation.c_str());

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
