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

std::string roomNameROSParam;

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

void roomToStruct(std::string fileName) {

}

int main(int argc, char const *argv[]) {
	std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
	objectsFileLoc = wheelchair_dump_loc + "/dump/mobilenet/" + roomNameROSParam + mobilenetFileType;
	//weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/" + roomNameROSParam + weightingFileType;
	weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/";
	roomListLoc = wheelchair_dump_loc + "/dump/context_training/room.list";

	roomToStruct(roomListLoc);

	return 0;
}