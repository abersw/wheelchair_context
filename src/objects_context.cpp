/*
 * detected_objects_context.cpp
 * wheelchair_context
 * version: 0.1.0 Majestic Maidenhair
 * Status: Beta
 * 
*/

#include "tof_tool/tof_tool_box.h"

#include "wheelchair_msgs/objectLocations.h"
#include "wheelchair_msgs/objectContext.h"

using namespace std;

static const int DEBUG_contextListToStruct = 0;
static const int DEBUG_calculateInfluenceWeight = 0;
static const int DEBUG_listToContextInfo = 0;
static const int DEBUG_objectLocationsCallback = 1;
static const int DEBUG_main = 0;
static const int DEBUG_fileLocations = 1;

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
};
struct Objects objectsFileStruct[100000]; //array for storing object data
int totalObjectsFileStruct = 0; //total objects inside struct

struct Context {
    int object_id; //object id
    string object_name; //object name
    float object_confidence; //object confidence from dnn
    int object_detected; //times object has been detected

    double object_weighting; //object weighting result
    double object_uniqueness; //object uniqueness result
    double object_score; //calculation of object weighting and uniqueness
    int object_instances; //number of objects in env

    int objectDetectedFlag = 0; //turns to 1 if object has been detected when driving around
};
//object_id,object_name,object_confidence,object_detected,object_weighting,object_uniqueness,object_instances
struct Context objectContext[100000]; //struct for storing object context info
int totalObjectContextStruct = 0; //total objects in struct

struct TrainingInfo {
    int times_trained; //real times trained
    int times_trained_max = 5; //value to prevent times trained val becoming too small
    double times_trained_val; //actual value used for calculating object weighting
    int max_weighting = 1; //max value for object weighting
    int min_weighting = 0; //min value for object weighting
    int max_uniqueness = 1; //max value for object uniqueness
    int min_uniqueness = 0; //min value for object uniqueness
};
struct TrainingInfo trainingInfo;

//list of file locations
std::string wheelchair_dump_loc; //location of wheelchair_dump package
std::string dump_context_loc = "/dump/context/"; //location of context dir in wheelchair_dump
std::string context_list_name = "objects.context"; //name of object context file
std::string context_info_name = "info.context"; //name of context training info file
std::string context_list_loc; //full path to object context file
std::string context_info_loc; //full path to context training info file

TofToolBox *tofToolBox;

/**
 * Function to add context data from param 'fileName' path, start assigning info from each line of file
 *
 * @param pass the path and file name to be created called 'fileName'
 */
void contextListToStruct(std::string fileName) {
    std::string objectsDelimiter = ","; //delimiter character is comma
	ifstream FILE_READER(fileName); //open file
    int objectNumber = 0; //iterate on each object
    if (FILE_READER.peek() == std::ifstream::traits_type::eof()) {
        //don't do anything if next character in file is eof
        cout << "file is empty, struct will remain empty" << endl;
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
                    objectContext[objectNumber].object_detected = std::stod(token); //set times object has been detected
                }
                else if (lineSection == 4) {
                    objectContext[objectNumber].object_weighting = std::stod(token); //set object weighting
                }
                else if (lineSection == 5) {
                    objectContext[objectNumber].object_uniqueness = std::stod(token); //set object uniqueness
                }
                lineSection++; //move to next element in line
            }
            objectContext[objectNumber].object_instances = std::stod(line); //set object instances
            if (DEBUG_contextListToStruct) { //print off debug lines
                cout << "sections in line " << lineSection << endl;
                cout << objectContext[objectNumber].object_id << "," << objectContext[objectNumber].object_name << ", " << objectContext[objectNumber].object_confidence << endl;
                cout << objectContext[objectNumber].object_detected << ", " << objectContext[objectNumber].object_weighting << ", " << objectContext[objectNumber].object_uniqueness << ", " << objectContext[objectNumber].object_instances << endl;
                tofToolBox->printSeparator(0);
            }
            objectNumber++; //iterate to next object in list
        }
    }
    totalObjectContextStruct = objectNumber; //var to add number of objects in struct
}

/**
 * influence weight I is calculated as the inverse of
   the number of times the navigation software has been launched
 *
 * @param pass L: times training times (software launched)
 */
double calculateInfluenceWeight() {
    double influenceWeight = 0.0;
    if (trainingInfo.times_trained <= trainingInfo.times_trained_max) { //if times trained is less than or equal to max times trained
        if (DEBUG_calculateInfluenceWeight) {
            cout << "training times in range" << endl;
        }
        influenceWeight = 1.0 / trainingInfo.times_trained;
        trainingInfo.times_trained_val = influenceWeight;
    }
    else if (trainingInfo.times_trained > trainingInfo.times_trained_max) { //if actual times trained is greater than max times trained
        if (DEBUG_calculateInfluenceWeight) {
            cout << "training times out of range 5" << endl;
        }
        influenceWeight = 1.0 / trainingInfo.times_trained_max;
        trainingInfo.times_trained_val = influenceWeight; //assign max times trained to calculation values
    }
    else { //throw error if input form context file is invalid
        cout << "invalid times trained from context info file" << endl;
    }
    if (DEBUG_calculateInfluenceWeight) {
        cout << "Influence (I) is " << influenceWeight << endl;
    }
    return influenceWeight;
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
        calculateInfluenceWeight();
    }
    else {
        std::string line;
        while (getline(FILE_READER, line)) { //go through line by line
            //times trained read in and assign to struct
            if (lineNumber == 0) {
                //line 0 contains times trained
                int getTimesTrained = std::stoi(line);
                trainingInfo.times_trained = getTimesTrained + 1; //add one to times trained on startup
                calculateInfluenceWeight(); //calculate if influence weighting needs to be capped at max
                if (DEBUG_listToContextInfo) {
                    cout << "training session is " << trainingInfo.times_trained << endl;
                    cout << "training value used is " << trainingInfo.times_trained_val << endl;
                }
            }
            lineNumber++; //iterate to next line
        }
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
    totalObjectsFileStruct = totalObjectsInMsg; //set message total objects to total objects in file struct
    if (DEBUG_objectLocationsCallback) {
        tofToolBox->printSeparator(0);
    }
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

        if (DEBUG_objectLocationsCallback) {
            cout << objectsFileStruct[isObject].id << ":" << objectsFileStruct[isObject].object_name << endl;
        }
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
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    //take UID from publish_objects_location and pass it through here
    //when msg comes through with ID of object - append a room name to the object
    ros::init(argc, argv, "objects_context");
    ros::NodeHandle n;

    wheelchair_dump_loc = tofToolBox->doesPkgExist("wheelchair_dump");//check to see if dump package exists
    context_list_loc = wheelchair_dump_loc + dump_context_loc + context_list_name; //concatenate vars to create location of room list
    if (DEBUG_fileLocations) {
        cout << "file location is " << context_list_loc << endl;
    }
    tofToolBox->createFile(context_list_loc); //check to see if file is present, if not create a new one
    contextListToStruct(context_list_loc); //add list to struct

    context_info_loc = wheelchair_dump_loc + dump_context_loc + context_info_name; //concatenate vars to create location of room list
    if (DEBUG_fileLocations) {
        cout << "file location is " << context_info_loc << endl;
    }
    tofToolBox->createFile(context_info_loc); //check to see if file is present, if not create a new one
    listToContextInfo(context_info_loc); //set context training info to struct

    ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 1000, objectLocationsCallback); //full list of objects

    ros::Rate rate(10.0);
    while(ros::ok()) {
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}