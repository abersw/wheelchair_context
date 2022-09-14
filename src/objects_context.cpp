/*
 * objects_context.cpp
 * wheelchair_context
 * version: 0.1.0 Majestic Maidenhair
 * Status: Beta
 * 
*/

#include "tof_tool/tof_tool_box.h"

#include "wheelchair_msgs/objectLocations.h"
#include "wheelchair_msgs/objectContext.h"
#include "wheelchair_msgs/missingObjects.h"
#include "wheelchair_msgs/trackingContext.h"

#include <ros/callback_queue.h>
#include <thread>
#include <chrono>

using namespace std;

static const int DEBUG_trackingFileToArray = 0;
static const int DEBUG_populateObjectsToTrack = 0;
static const int DEBUG_listenForTrackingObjects = 0;
static const int DEBUG_captureTrackingObject = 0;
static const int DEBUG_trackingObjectFound = 0;
static const int DEBUG_contextListToStruct = 0;
static const int DEBUG_calculateInfluenceWeight = 0;
static const int DEBUG_listToContextInfo = 0;
static const int DEBUG_addObjectToDictionary = 0;
static const int DEBUG_calculateObjectInstances = 0;
static const int DEBUG_calculateObjectUniqueness = 0;
static const int DEBUG_calculateContextScore = 0;
static const int DEBUG_publishObjectContext = 0;
static const int DEBUG_objectLocationsCallbackDictionary = 0;
static const int DEBUG_objectLocationsCallback = 0;
static const int DEBUG_assignObjectsDetectedStruct = 0;
static const int DEBUG_assignObjectsMissingStruct = 0;
static const int DEBUG_contextNoHistory = 0;
static const int DEBUG_contextMissingNoHistory = 0;
static const int DEBUG_contextWithHistory = 0;
static const int DEBUG_contextMissingWithHistory = 0;
static const int DEBUG_detectedObjectCallback = 0;
static const int DEBUG_missingObjectCallback = 0;
static const int DEBUG_contextInfoToList = 0;
static const int DEBUG_contextStructToList = 0;
static const int DEBUG_currentTime = 0;
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

static const int numOfPastFrames = 5;
struct Objects objectsDetectedStruct[numOfPastFrames][1000]; //5 previous frames, 1000 potential objects
int totalObjectsDetectedStruct[numOfPastFrames]; //size of struct for previous 5 frames

struct MissingObjects { //struct for publishing topic
    int id; //get object id from ros msg
    string object_name; //get object name/class
    int totalCorrespondingPoints; //get total corresponding points from pointcloud
};

struct MissingObjects objectsMissingStruct[numOfPastFrames][1000]; //5 previous frames, 1000 potential objects
int totalObjectsMissingStruct[numOfPastFrames]; //size of struct for previous 5 frames

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
    double max_weighting = 1.0; //max value for object weighting
    double min_weighting = 0.0; //min value for object weighting
    double max_uniqueness = 1.0; //max value for object uniqueness
    double min_uniqueness = 0.0; //min value for object uniqueness
};
struct TrainingInfo trainingInfo;

//struct will store single object names and the instances inside the entire environment
struct ObjectDictionary {
    std::string object_name; //object name
    int instances; //instances of object in environment
};
struct ObjectDictionary objectDictionary[1000]; //struct for storing data needed to calc uniqueness of objects
int totalObjectDictionaryStruct = 0; //total list of objects used to calc uniqueness

//context data to save
struct TrackingObjects {
    int object_id;
    string object_name;
    float object_confidence; //object confidence from dnn

    double object_timestamp; //should be saved in seconds .toSec()
    double duration; //duration from start of node launch

    //context info
    int times_trained; //real times trained
    double times_trained_val; //actual value used for calculating object weighting

    //context data
    int object_detected; //times object has been detected

    double object_weighting; //object weighting result
    double object_uniqueness; //object uniqueness result
    double object_score; //calculation of object weighting and uniqueness
    int object_instances; //number of objects in env
};
static const int totalObjectsTracked = 1000;
static const long totalObjectsTrackedCaptured = 10000;
//[0] contains object id and name [1] instances detected
struct TrackingObjects trackingObjects[totalObjectsTracked][totalObjectsTrackedCaptured];

//contains instances of object found, uses order from trackingObjects
int totalTrackingObjectsCaptured[totalObjectsTracked];

struct TrackingObjects trackingObjectsList[totalObjectsTracked];
//std::map<string, string> trackingObjectsListRaw = {{"42", "refrigerator"}, {"53", "refrigerator"}};
//string trackingObjectsListRaw[] = {"4", "backpack", "56", "refrigerator", "59", "oven"};
string trackingObjectsListRaw[10000];
int totalTrackingObjectsListRaw = 0;
int totalTrackingObjectsList = 0;
static const int TRACKING_ID_RANGE = 3;

double currentTimeSecs = 0.0;
double PARAM_add_to_duration = 0.0;

int checkTimeOnStartup = 0;

//list of file locations
std::string wheelchair_dump_loc; //location of wheelchair_dump package
std::string dump_context_loc = "/dump/context/"; //location of context dir in wheelchair_dump
std::string context_list_name = "objects.context"; //name of object context file
std::string context_info_name = "info.context"; //name of context training info file
std::string context_list_loc; //full path to object context file
std::string context_info_loc; //full path to context training info file

std::string wheelchair_experiments_loc; //location of wheelchair_dump package
std::string experiments_loc = "/docs/objects-to-track/"; //location of list of objects to tack
std::string experiments_loc_file;

ros::Publisher *ptr_object_context;
ros::Publisher *ptr_tracking_context;

TofToolBox *tofToolBox;

int contextIsDetected = 1;
int contextIsMissing = 0;

static const int saveDataToList = 1;

int objectsToTrack = 0;

double beginTime = 0.0;


void trackingFileToArray() {
    int counter = 0;
    std::ifstream file(experiments_loc_file);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            // using printf() in all tests for consistency
            if (DEBUG_trackingFileToArray) {
                cout << "complete line from file is " << line << endl;
            }
            size_t colon_pos = line.find(':');
            string str1 = line.substr(0, colon_pos);
            string str2 = line.substr(colon_pos+1);
            trackingObjectsListRaw[counter] = str1;
            counter++;
            trackingObjectsListRaw[counter] = str2;
            counter++;
        }
        file.close();
    }
    else {
        cout << "something went wrong opening the file" << endl;
    }
    totalTrackingObjectsListRaw = counter;
    if (DEBUG_trackingFileToArray) {
        for (int i = 0; i < counter; i++) {
            cout << trackingObjectsListRaw[i] << endl;
        }
    }
}

void populateObjectsToTrack() {
    if (DEBUG_populateObjectsToTrack) {
        cout << "total tracking objects list raw " << totalTrackingObjectsListRaw << endl;
    }
    int pos = 0;
    int counter = 0;

    int targetObjectID = -1;
    std::string targetObjectName = "";

    for (int i = 0; i < totalTrackingObjectsListRaw; i++) {
        if (pos == 0) {
            targetObjectID = std::stoi(trackingObjectsListRaw[i]);
            pos++;
        }
        else if (pos == 1) {
            targetObjectName = trackingObjectsListRaw[i];
            pos = 0; //next element in list will be id

            //track object IDs within specified range
            //run from min range of object ID
            for (int iterateObjectID = (targetObjectID - TRACKING_ID_RANGE); iterateObjectID < targetObjectID; iterateObjectID++) {
                //std::cout << "run less " << iterateObjectID << std::endl;
                trackingObjects[counter][0].object_id = iterateObjectID;
                trackingObjects[counter][0].object_name = targetObjectName;
                totalTrackingObjectsCaptured[counter] = 0;
                counter++;
            }
            //run from object id to max range
            for (int iterateObjectID = targetObjectID; iterateObjectID <= (targetObjectID + TRACKING_ID_RANGE); iterateObjectID++) {
                //std::cout << "run more " << iterateObjectID << std::endl;
                trackingObjects[counter][0].object_id = iterateObjectID;
                trackingObjects[counter][0].object_name = targetObjectName;
                totalTrackingObjectsCaptured[counter] = 0;
                counter++;
            }
        }
        else {
            if (DEBUG_populateObjectsToTrack) {
                cout << "something went wrong during allocation" << endl;
            }
        }
    }
    totalTrackingObjectsList = counter;

    if (DEBUG_populateObjectsToTrack) {
        cout << "total objects to track is " << totalTrackingObjectsList << endl;
        for (int i = 0; i < totalTrackingObjectsList; i++) {
            cout << trackingObjects[i][0].object_id << ":" << trackingObjects[i][0].object_name << endl;
        }
    }
}

std::pair<int , int> listenForTrackingObjects(int currentObjectID, string currentObjectName) {
    //loop through tracked list, return true if object
    int trackedObjectFound = 0;
    int trackedObjectPos = 0;
    for (int isTrackingObject = 0; isTrackingObject < totalTrackingObjectsList; isTrackingObject++) {
        if ((currentObjectID == trackingObjects[isTrackingObject][0].object_id) &&
            (currentObjectName == trackingObjects[isTrackingObject][0].object_name)) {
            trackedObjectFound = 1;
            trackedObjectPos = isTrackingObject;
            if (DEBUG_listenForTrackingObjects) {
                cout << "found object in tracking " << currentObjectID << ":" << currentObjectName << " in pos " << isTrackingObject << endl;
            }
        }
    }
    return std::make_pair(trackedObjectFound, trackedObjectPos);
}

void captureTrackingObject(int isDetectedObject, int trackingObjectPos, int currentObjectID, string currentObjectName) {
    //get all information and apply to struct
    //find position of object in context array
    int objectContextPos = 0;
    //cout << "total context struct is " << totalObjectContextStruct << endl;
    for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) {
        if ((currentObjectID == objectContext[isContext].object_id) &&
            (currentObjectName == objectContext[isContext].object_name)) {
            //cout << "context match found at pos" << isContext << endl;
            objectContextPos = isContext;
        }
    }
    if (DEBUG_captureTrackingObject) {
        cout << "trackingObjectPos is " << trackingObjectPos << endl;
        cout << "trackingObjectCaptured is " << totalTrackingObjectsCaptured[trackingObjectPos] << endl;
    }
    if (objectContext[objectContextPos].object_instances == 0) {
        ROS_ERROR_STREAM(to_string(objectContext[objectContextPos].object_id) + objectContext[objectContextPos].object_name);
        ROS_ERROR_STREAM("something has gone very wrong, detected 0 instances");
    }

    wheelchair_msgs::trackingContext trackObj; //initialise ros message type
    trackObj.header.stamp = ros::Time::now();
    trackObj.object_id = objectContext[objectContextPos].object_id;
    trackObj.object_name = objectContext[objectContextPos].object_name;
    trackObj.object_confidence = objectContext[objectContextPos].object_confidence;
    trackObj.object_detected = objectContext[objectContextPos].object_detected;

    trackObj.object_weighting = objectContext[objectContextPos].object_weighting;
    trackObj.object_uniqueness = objectContext[objectContextPos].object_uniqueness;
    trackObj.object_score = objectContext[objectContextPos].object_score;
    trackObj.object_instances = objectContext[objectContextPos].object_instances;

    trackObj.object_timestamp = currentTimeSecs;

    trackObj.duration = (currentTimeSecs - beginTime) + PARAM_add_to_duration;

    trackObj.detected_or_missing = isDetectedObject;

    ptr_tracking_context->publish(trackObj);
}

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
            objectContext[objectNumber].object_instances = std::stoi(line); //set object instances
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
 * Function to create a dictionary of objects from the main objects array
 *
 */
void addObjectToDictionary() {
    totalObjectContextStruct = totalObjectsFileStruct; //set object context struct size to replicate size of object struct
    for (int isObject = 0; isObject < totalObjectContextStruct; isObject++) { //run through all objects
        int objectMatched = 0; //flag variable sets to 1 if object is already in dictionary
        std::string getObjName = objectsFileStruct[isObject].object_name; //get object name from objectsFileStruct
        if (totalObjectDictionaryStruct == 0) {
            objectDictionary[0].object_name = getObjName; //set object name in first element from full objects struct
            totalObjectDictionaryStruct++; //add 1 to total objects in dictionary
        }
        for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) { //iterate through dictionary struct
            std::string getObjDictName = objectDictionary[isDict].object_name; //get object name from objectDictionary
            if (getObjName == getObjDictName) { //if name from objectsFileStruct and objectDictionary is the same
                objectMatched = 1; //set to true
            }
            //objectDictionary[isDict].instances = 0; //set objects back to 0
            //cout << "DEBUG: adding zero for " << getObjName << endl;
        }
        if (objectMatched) {
            //if object is already in struct, don't add anything
            //instances calculated in calculateObjectInstances
        }
        else {
            //add object name to struct
            objectDictionary[totalObjectDictionaryStruct].object_name = getObjName; //assign name from objectsFileStruct
            objectDictionary[totalObjectDictionaryStruct].instances = 0; //set instances to 0
            totalObjectDictionaryStruct++; //add for next element in array
        }
    }
    //print out list of objects
    if (DEBUG_addObjectToDictionary) {
        tofToolBox->printSeparator(1);
        cout << "pre-instance calculations, total size of struct is " << totalObjectDictionaryStruct << endl;
        for (int isDet = 0; isDet < totalObjectDictionaryStruct; isDet++) {
            cout << objectDictionary[isDet].object_name << ":" << objectDictionary[isDet].instances << endl;
        }
        tofToolBox->printSeparator(1);
    }
}

/**
 * Function to calculate object instances from dictionary of objects
 *
 */
void calculateObjectInstances() {
    for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) { //iterate through object dictionary
        std::string getObjDictName = objectDictionary[isDict].object_name; //get object name from dictionary
        if (DEBUG_calculateObjectInstances) {
            tofToolBox->printSeparator(1);
            cout << "total objects in dictionary is " << totalObjectDictionaryStruct << endl;
            cout << "object from dict is " << getObjDictName << endl;
        }
        for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) { //iterate through object struct
            std::string getObjName = objectContext[isContext].object_name; //get object name from main struct
            if (DEBUG_calculateObjectInstances) {
                cout << "total objects in context is " << totalObjectContextStruct << endl;
                cout << "total objects in struct is " << totalObjectsFileStruct << endl;
                cout << "object from context is " << getObjName << endl;
            }
            if (getObjDictName == getObjName) { //if object name in dictionary and main struct are equal
                if (DEBUG_calculateObjectInstances) {
                    cout << "found instance" << endl;
                }
                objectDictionary[isDict].instances++; //add 1 to object instances
            }
            else {
                //don't do anything if match not found between dictionary and main object struct
            }
        }
    }
    //print out list and instances of objects
    if (DEBUG_objectLocationsCallbackDictionary) {
        for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) {
            cout << objectDictionary[isDict].object_name << ":" << objectDictionary[isDict].instances << endl;
        }
    }
}

double calculateObjectUniqueness(int isDict) {
    double currentObjectUniqueness = 0.0;
    double calculatedUniqueness = 1.0 / objectDictionary[isDict].instances;
    if ((calculatedUniqueness <= trainingInfo.max_uniqueness) && (calculatedUniqueness >= trainingInfo.min_uniqueness)) {
        currentObjectUniqueness = calculatedUniqueness;
        if (DEBUG_calculateObjectUniqueness) {
            cout << "object uniqueness is in range: " << currentObjectUniqueness << endl;
        }
    }
    else if (calculatedUniqueness > trainingInfo.max_uniqueness) {
        currentObjectUniqueness = trainingInfo.max_uniqueness;
        if (DEBUG_calculateObjectUniqueness) {
            cout << "object uniqueness is larger than range, " << calculatedUniqueness << " reverted to : " << currentObjectUniqueness << endl;
        }
    }
    else if (calculatedUniqueness < trainingInfo.min_uniqueness) {
        currentObjectUniqueness = trainingInfo.min_uniqueness;
        if (DEBUG_calculateObjectUniqueness) {
            cout << "object uniqueness is below range, " << calculatedUniqueness << " reverted to : " << currentObjectUniqueness << endl;
        }
    }
    else { //throw error if input form context file is invalid
        cout << "invalid times trained from context info file" << endl;
    }
    if (DEBUG_calculateObjectUniqueness) {
        cout << "Uniqueness (U) is " << objectDictionary[isDict].object_name << ":" << currentObjectUniqueness << endl;
    }
    return currentObjectUniqueness;
}

/**
 * Function to calculate the object context score
 *
 * @param parameter 'isContext' is the current position in the array objectContext
 *        object is called from the objectLocationsCallback function
 */
void calculateContextScore(int isContext) {
    objectContext[isContext].object_score =
    objectContext[isContext].object_weighting *
    objectContext[isContext].object_uniqueness *
    objectContext[isContext].object_confidence;

    //print the current object context data
    if (DEBUG_calculateContextScore) {
        cout << "pos is " << isContext <<
        ", object name: " << objectContext[isContext].object_name <<
        ", object weighting: " << objectContext[isContext].object_weighting <<
        ", object uniqueness: " << objectContext[isContext].object_uniqueness <<
        ", object instances: " << objectContext[isContext].object_instances <<
        ", object score: " << objectContext[isContext].object_score << endl;
    }
}

/**
 * Function to get all data required to calculate context, calls calculate context
 *
 */
void getObjectContext() {
    for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) {
        std::string getObjDictName = objectDictionary[isDict].object_name; //get object name from dictionary
        int getObjDictInstances = objectDictionary[isDict].instances; //get instances from object dictionary
        double currentObjDictUniqueness = calculateObjectUniqueness(isDict); //main calculation for uniqueness
        for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) { //iterate through entire context struct
            std::string getObjName = objectContext[isContext].object_name;
            if (getObjDictName == getObjName) {
                objectContext[isContext].object_uniqueness = currentObjDictUniqueness; //assign current object uniqueness
                objectContext[isContext].object_instances = getObjDictInstances; //assign instances of objects
                calculateContextScore(isContext); //calculate object context score
            }
            else {
                //don't do anything if objects don't match
            }
        }
    }
}

/**
 * Publish context data as ROS msg array
 *
 */
void publishObjectContext() {
    wheelchair_msgs::objectContext objContext;

    for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) {
        if (DEBUG_publishObjectContext) {
            cout <<
            objectContext[isContext].object_id << ", " <<
            objectContext[isContext].object_name << ", " <<
            objectContext[isContext].object_confidence << ", " <<
            objectContext[isContext].object_detected << ", " <<

            objectContext[isContext].object_weighting << ", " <<
            objectContext[isContext].object_uniqueness << ", " <<
            objectContext[isContext].object_score << ", " <<
            objectContext[isContext].object_instances << endl;
        }
        objContext.object_id.push_back(objectContext[isContext].object_id);
        objContext.object_name.push_back(objectContext[isContext].object_name);
        objContext.object_confidence.push_back(objectContext[isContext].object_confidence);
        objContext.object_detected.push_back(objectContext[isContext].object_detected);

        objContext.object_weighting.push_back(objectContext[isContext].object_weighting);
        objContext.object_uniqueness.push_back(objectContext[isContext].object_uniqueness);
        objContext.object_score.push_back(objectContext[isContext].object_score);
        objContext.object_instances.push_back(objectContext[isContext].object_instances);
    }
    objContext.totalObjects = totalObjectContextStruct;
    ptr_object_context->publish(objContext);
}

/**
 * Main callback function triggered by received ROS topic
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    if (checkTimeOnStartup == 0) {
        beginTime = ros::Time::now().toSec();
        //n.setParam("/wheelchair_robot/context/tracking/begin_time", beginTime);
        cout << "start time is " << beginTime << endl;
        checkTimeOnStartup = 1;
    }
    
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

    //create and add object names to object dictionary struct
    addObjectToDictionary();

    //get object instances and assign to object dictionary struct
    calculateObjectInstances();

    //get data to calculate context
    getObjectContext();

    //publish calculated context via ROS msg
    publishObjectContext(); //publish object context data as ROS msg

    currentTimeSecs = ros::Time::now().toSec();
    if (DEBUG_currentTime) {
        cout.precision(12);
        cout << "current time is " << fixed << currentTimeSecs << endl;
    }
}

/**
 * Function to assign ROS topic msg context to struct
 * @param 'detPos' is the objects detected sequence used - 0 latest, 1 previous
 * @param 'detectedObject' object position in detected array
 * @param 'obLoc' belongs to wheelchair_msgs::objectLocations - contains object info
*/
void assignObjectsDetectedStruct(int detPos, const wheelchair_msgs::objectLocations obLoc, int detectedObject) {
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
    if (DEBUG_assignObjectsDetectedStruct) {
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

/**
 * Function to assign ROS topic msg context to struct
 * @param 'detPos' is the objects missing sequence used - 0 latest, 1 previous
 * @param 'missingObject' object position in missing array
 * @param 'misObj' belongs to wheelchair_msgs::missingObjects - contains object info
*/
void assignObjectsMissingStruct(int detPos, const wheelchair_msgs::missingObjects::ConstPtr& misObj, int missingObject) {
    objectsMissingStruct[detPos][missingObject].id = misObj->id[missingObject]; //assign object id to struct
    objectsMissingStruct[detPos][missingObject].object_name = misObj->object_name[missingObject]; //assign object name to struct
    objectsMissingStruct[detPos][missingObject].totalCorrespondingPoints = misObj->totalCorrespondingPoints[missingObject]; //assign object confidence to struct


    //objectsDetectedStruct[detPos][missingObject].inLastFrame; //don't do anything yet
    if (DEBUG_assignObjectsMissingStruct) {
        cout <<
        objectsMissingStruct[detPos][missingObject].id << "," <<
        objectsMissingStruct[detPos][missingObject].object_name << "," <<
        objectsMissingStruct[detPos][missingObject].totalCorrespondingPoints << endl;
    }
}

/**
 * Apply new object context weighting to struct
 *
 */
void applyNewWeighting(int isContext, double isNewWeighting) {
    if (DEBUG_detectedObjectCallback) {
        cout << "new weighting is " << isNewWeighting << endl;
    }
    if (isNewWeighting > trainingInfo.max_weighting) { //if outside of max weighting
        objectContext[isContext].object_weighting = trainingInfo.max_weighting; //assign object weighting max weight
        if (DEBUG_detectedObjectCallback) {
            cout << "set weighting to max: " << trainingInfo.max_weighting << endl;
        }
    }
    else if (isNewWeighting < trainingInfo.min_weighting) { //if outside of min weighting
        objectContext[isContext].object_weighting = trainingInfo.min_weighting; //assign object weighting min weight
        if (DEBUG_detectedObjectCallback) {
            cout << "set weighting to min: " << trainingInfo.min_weighting << endl;
        }
    }
    else { //if inside bounding weight
        objectContext[isContext].object_weighting = isNewWeighting; //assign object weighting caluclated weight
        if (DEBUG_detectedObjectCallback) {
            cout << "assigned to context struct pos " << isContext << " weighting " << objectContext[isContext].object_weighting << endl;
        }
    }
}

/**
 * Function to shift data in objects detected struct from pos 'from' to 'to'
 *
 * @param parameter 'from' is the position in the objects detected struct array where data is coming from
 * @param parameter 'to' is the position in the objects detected struct array where data is going to
 */
void shiftObjectsDetectedStructPos(int from, int to) {
    //shift data from pos 0 to pos 1, ready for next object detection callback
    for (int detectedObject = 0; detectedObject < totalObjectsDetectedStruct[from]; detectedObject++) {
        objectsDetectedStruct[to][detectedObject].id = objectsDetectedStruct[from][detectedObject].id; //assign object id to struct
        objectsDetectedStruct[to][detectedObject].object_name = objectsDetectedStruct[from][detectedObject].object_name; //assign object name to struct
        objectsDetectedStruct[to][detectedObject].object_confidence = objectsDetectedStruct[from][detectedObject].object_confidence; //assign object confidence to struct

        objectsDetectedStruct[to][detectedObject].point_x = objectsDetectedStruct[from][detectedObject].point_x; //assign object vector point x to struct
        objectsDetectedStruct[to][detectedObject].point_y = objectsDetectedStruct[from][detectedObject].point_y; //assign object vector point y to struct
        objectsDetectedStruct[to][detectedObject].point_z = objectsDetectedStruct[from][detectedObject].point_z; //assign object vector point z to struct

        objectsDetectedStruct[to][detectedObject].quat_x = objectsDetectedStruct[from][detectedObject].quat_x; //assign object quaternion x to struct
        objectsDetectedStruct[to][detectedObject].quat_y = objectsDetectedStruct[from][detectedObject].quat_y; //assign object quaternion y to struct
        objectsDetectedStruct[to][detectedObject].quat_z = objectsDetectedStruct[from][detectedObject].quat_z; //assign object quaternion z to struct
        objectsDetectedStruct[to][detectedObject].quat_w = objectsDetectedStruct[from][detectedObject].quat_w; //assign object quaternion w to struct
    }
    totalObjectsDetectedStruct[to] = totalObjectsDetectedStruct[from]; //set total objects in detection struct to pos 1
}

/**
 * Function to shift data in objects missing struct from pos 'from' to 'to'
 *
 * @param parameter 'from' is the position in the objects missing struct array where data is coming from
 * @param parameter 'to' is the position in the objects missing struct array where data is going to
 */
void shiftObjectsMissingStructPos(int from, int to) {
    //shift data from pos 0 to pos 1, ready for next object detection callback
    for (int missingObject = 0; missingObject < totalObjectsDetectedStruct[from]; missingObject++) {
        objectsMissingStruct[to][missingObject].id = objectsMissingStruct[from][missingObject].id; //assign object id to struct
        objectsMissingStruct[to][missingObject].object_name = objectsMissingStruct[from][missingObject].object_name; //assign object name to struct
        objectsMissingStruct[to][missingObject].totalCorrespondingPoints = objectsMissingStruct[from][missingObject].totalCorrespondingPoints; //assign total corresponding points to struct
    }
    totalObjectsMissingStruct[to] = totalObjectsMissingStruct[from]; //set total objects in detection struct to pos 1
}

/**
 * No previous history, so add to weighting
 *
 */
void contextNoHistory(int detPos) {
    for (int detectedObject = 0; detectedObject < totalObjectsDetectedStruct[detPos]; detectedObject++) { //run through struct of pos 0
        //run through each detected object
        int getDetObjID = objectsDetectedStruct[detPos][detectedObject].id; //get id
        std::string getDetObjName = objectsDetectedStruct[detPos][detectedObject].object_name; //get name

        for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) { //run through entire context struct
            int getContextID = objectContext[isContext].object_id; //get context ID
            std::string getContextName = objectContext[isContext].object_name; //get context name

            if ((getDetObjID == getContextID) && (getDetObjName == getContextName)) { //if object ID and name are equal
                if (DEBUG_contextNoHistory) {
                    tofToolBox->printSeparator(0);
                    cout << "found object " << getDetObjID << " in det and context struct" << endl;
                }
                //update object weighting and detected
                objectContext[isContext].object_detected++; //add one to times object was detected in env
                //objectContext[isContext].objectDetectedFlag = 1; //object has been found, assign 1 to flag
                double isCurrentWeighting = objectContext[isContext].object_weighting;
                double isNewWeighting = isCurrentWeighting + trainingInfo.times_trained_val;
                applyNewWeighting(isContext, isNewWeighting);
                //get data to calculate context
                getObjectContext();
                std::pair<int, int> listenForTrackingObjectsResult = listenForTrackingObjects(getDetObjID, getDetObjName);
                int trackingObjectFound = listenForTrackingObjectsResult.first;
                int trackingObjectPos = listenForTrackingObjectsResult.second;
                if (trackingObjectFound == 1) {
                    if (DEBUG_trackingObjectFound) {
                        cout << "contextNoHistory" << endl;
                        cout << "tracking object " << getDetObjID << ":" << getDetObjName << " found" << endl;
                    }
                    captureTrackingObject(contextIsDetected, trackingObjectPos, getDetObjID, getDetObjName);
                }
            }

        }
    }
    shiftObjectsDetectedStructPos(0,1); //shift detection data from struct pos 0 to 1
}

/**
 * No previous history, so subtract from weighting
 *
 */
void contextMissingNoHistory(int detPos) {
    for (int missingObject = 0; missingObject < totalObjectsMissingStruct[detPos]; missingObject++) { //run through struct of pos 0
        //run through each detected object
        int getDetObjID = objectsMissingStruct[detPos][missingObject].id; //get id
        std::string getDetObjName = objectsMissingStruct[detPos][missingObject].object_name; //get name

        for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) { //run through entire context struct
            int getContextID = objectContext[isContext].object_id; //get context ID
            std::string getContextName = objectContext[isContext].object_name; //get context name

            if ((getDetObjID == getContextID) && (getDetObjName == getContextName)) { //if object ID and name are equal
                if (DEBUG_contextNoHistory) {
                    tofToolBox->printSeparator(0);
                    cout << "missing object " << getDetObjID << " in det and context struct" << endl;
                }
                //update object weighting
                //objectContext[isContext].object_detected++; //don't add to object detected if object is missing
                //objectContext[isContext].objectDetectedFlag = 1; //object has been found, assign 1 to flag
                double isCurrentWeighting = objectContext[isContext].object_weighting;
                double isNewWeighting = isCurrentWeighting - trainingInfo.times_trained_val;
                applyNewWeighting(isContext, isNewWeighting);
                //get data to calculate context
                getObjectContext();
                std::pair<int, int> listenForTrackingObjectsResult = listenForTrackingObjects(getDetObjID, getDetObjName);
                int trackingObjectFound = listenForTrackingObjectsResult.first;
                int trackingObjectPos = listenForTrackingObjectsResult.second;
                if (trackingObjectFound == 1) {
                    if (DEBUG_trackingObjectFound) {
                        cout << "contextMissingNoHistory" << endl;
                        cout << "tracking object " << getDetObjID << ":" << getDetObjName << " found" << endl;
                    }
                    captureTrackingObject(contextIsMissing, trackingObjectPos, getDetObjID, getDetObjName);
                }
                cout << "object weighting has been reduced to " << objectContext[isContext].object_weighting << endl;
            }

        }
    }
    shiftObjectsMissingStructPos(0,1); //shift detection data from struct pos 0 to 1
}

/**
 * History exists, therefore compare with history to see if object was in previous frame
 *
 */
void contextWithHistory() {
    //run through latest stuct of detected objects
    for (int detectedObject = 0; detectedObject < totalObjectsDetectedStruct[0]; detectedObject++) {
        int getDetObjID = objectsDetectedStruct[0][detectedObject].id; //get id
        std::string getDetObjName = objectsDetectedStruct[0][detectedObject].object_name; //get name

        int objectFoundInHistory = 0;
        //run through previous struct of detected objects
        for (int lastDetectedObject = 0; lastDetectedObject < totalObjectsDetectedStruct[1]; lastDetectedObject++) {
            int getLastObjID = objectsDetectedStruct[1][lastDetectedObject].id; //get id
            std::string getLastObjName = objectsDetectedStruct[1][lastDetectedObject].object_name; //get name

            if ((getDetObjID == getLastObjID) && (getDetObjName == getLastObjName)) {
                objectFoundInHistory = 1; //object has been detected in previous history
            }
            else {
                //leave objectFoundInHistory as 0
            }
        }
        //finished running through history, was a match found?
        if (objectFoundInHistory) {
            //a match was found, do not recalculate weighting - because it must be the same object in the frame
        }
        else {
            //a match was not found
            //run through entire context struct for returning correct object id position
            for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) {
                int getContextID = objectContext[isContext].object_id; //get context ID
                std::string getContextName = objectContext[isContext].object_name; //get context name

                //if object ID and name are equal
                if ((getDetObjID == getContextID) && (getDetObjName == getContextName)) {
                    if (DEBUG_contextWithHistory) {
                        tofToolBox->printSeparator(0);
                        cout << "found object " << getDetObjID << " in both history structs" << endl;
                    }
                    //if new object in detected struct pos 0 is equal to object in context
                    //update object weighting and detected
                    objectContext[isContext].object_detected++; //add one to times object was detected in env
                    double isCurrentWeighting = objectContext[isContext].object_weighting;
                    double isNewWeighting = isCurrentWeighting + trainingInfo.times_trained_val;
                    applyNewWeighting(isContext, isNewWeighting);
                    //get data to calculate context
                    getObjectContext();
                    std::pair<int, int> listenForTrackingObjectsResult = listenForTrackingObjects(getDetObjID, getDetObjName);
                    int trackingObjectFound = listenForTrackingObjectsResult.first;
                    int trackingObjectPos = listenForTrackingObjectsResult.second;
                    if (trackingObjectFound == 1) {
                        if (DEBUG_trackingObjectFound) {
                            cout << "contextWithHistory" << endl;
                            cout << "tracking object " << getDetObjID << ":" << getDetObjName << " found" << endl;
                        }
                        captureTrackingObject(contextIsDetected, trackingObjectPos, getDetObjID, getDetObjName);
                    }
                }
            }
        }
    }
    shiftObjectsDetectedStructPos(0,1); //shift detection data from struct pos 0 to 1
}

/**
 * History exists, therefore compare with history to see if object was in previous frame
 *
 */
void contextMissingWithHistory() {
    //run through latest stuct of detected objects
    for (int missingObject = 0; missingObject < totalObjectsMissingStruct[0]; missingObject++) { //run through struct of detected objects
        int getMisObjID = objectsMissingStruct[0][missingObject].id; //get id
        std::string getMisObjName = objectsMissingStruct[0][missingObject].object_name; //get name

        int objectFoundInHistory = 0;
        //run through previous struct of detected objects
        for (int lastMissingObject = 0; lastMissingObject < totalObjectsMissingStruct[1]; lastMissingObject++) {
            int getLastObjID = objectsMissingStruct[1][lastMissingObject].id; //get id
            std::string getLastObjName = objectsMissingStruct[1][lastMissingObject].object_name; //get name

            if ((getMisObjID == getLastObjID) && (getMisObjName == getLastObjName)) {
                objectFoundInHistory = 1; //object has been detected in previous history
            }
            else {
                //leave objectFoundInHistory as 0
            }
        }
        //finished running through history, was a match found?
        if (objectFoundInHistory) {
            //a match was found, do not recalculate weighting - because it must be the same object in the frame
        }
        else {
            //a match was not found
            //run through entire context struct for returning correct object id position
            for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) {
                int getContextID = objectContext[isContext].object_id; //get context ID
                std::string getContextName = objectContext[isContext].object_name; //get context name

                //if object ID and name are equal
                if ((getMisObjID == getContextID) && (getMisObjName == getContextName)) {
                    if (DEBUG_contextMissingWithHistory) {
                        tofToolBox->printSeparator(0);
                        cout << "found object " << getMisObjID << " in both history structs" << endl;
                    }
                    //if missing object struct pos 0 is equal to object in context
                    //update object weighting and detected
                    double isCurrentWeighting = objectContext[isContext].object_weighting;
                    double isNewWeighting = isCurrentWeighting - trainingInfo.times_trained_val; //reduce weighting
                    applyNewWeighting(isContext, isNewWeighting);
                    //get data to calculate context
                    getObjectContext();

                    std::pair<int, int> listenForTrackingObjectsResult = listenForTrackingObjects(getMisObjID, getMisObjName);
                    int trackingObjectFound = listenForTrackingObjectsResult.first;
                    int trackingObjectPos = listenForTrackingObjectsResult.second;
                    if (trackingObjectFound == 1) {
                        if (DEBUG_trackingObjectFound) {
                            cout << "contextMissingWithHistory" << endl;
                            cout << "tracking object " << getMisObjID << ":" << getMisObjName << " found" << endl;
                        }
                        captureTrackingObject(contextIsMissing, trackingObjectPos, getMisObjID, getMisObjName);
                    }
                }
            }
        }
    }
    shiftObjectsMissingStructPos(0,1); //shift detection data from struct pos 0 to 1
}

/**
 * Main callback function triggered by detected objects in frame ROS topic
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void detectedObjectCallback(const wheelchair_msgs::objectLocations obLoc) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); //wait 200 milliseconds in thread for all objects to update
    if (DEBUG_detectedObjectCallback) {
        tofToolBox->printSeparator(1);
        cout << "new objects detected message received" << endl;
    }
    int detPos = 0; //'detPos' is the objects detected sequence used - 0 latest, 1 previous
    totalObjectsDetectedStruct[detPos] = obLoc.totalObjects; //total detected objects in ROS msg
    for (int detectedObject = 0; detectedObject < totalObjectsDetectedStruct[detPos]; detectedObject++) {
        //add to struct position [0][object number]
        assignObjectsDetectedStruct(detPos, obLoc, detectedObject); //assign ROS topic msg to struct
    }
    //finished adding detected data to pos 0 in 2d array

    //recalculate influence weighting for training session
    calculateInfluenceWeight();

    //start off with first object detections in sequence
    if (totalObjectsDetectedStruct[detPos+1] == 0) { //if number of objects in next element is 0, no history exists
        //no history to compare with, therefore do all the calculation stuff
        if (DEBUG_detectedObjectCallback) {
            cout << "nothing in struct pos " << detPos+1 << endl;
        }
        //first work out whether to add or delete from existing weighting value
        //no previous history, so add to weighting
        contextNoHistory(detPos);
    }
    else {
        //history exists, therefore compare with history to see if object was in previous frame
        if (DEBUG_detectedObjectCallback) {
            cout << "data exists in struct pos " << detPos+1 << endl;
        }
        contextWithHistory();
    }
    //calculate uniqueness here would probably work - doesn't need to detect an object to calculate - probably quicker too...
}

void missingObjectCallback(const wheelchair_msgs::missingObjects::ConstPtr& misObj) {
    if (DEBUG_missingObjectCallback) {
        tofToolBox->printSeparator(1);
        cout << "new missing objects message received" << endl;
    }
    int detPos = 0; //'detPos' is the missing objects message sequence used - 0 latest, 1 previous
    totalObjectsMissingStruct[detPos] = misObj->totalObjects; //total detected objects in ROS msg
    if (DEBUG_missingObjectCallback) {
        cout << "total missing objects in msg are " << totalObjectsMissingStruct[detPos] << endl;
    }
    for (int isMissingObject = 0; isMissingObject < totalObjectsMissingStruct[detPos]; isMissingObject++) {
        //cout << "missing object detected " << misObj->id[isMissingObject] << ":" << misObj->object_name[isMissingObject] << endl;
        //add to struct position [0][object number]
        assignObjectsMissingStruct(detPos, misObj, isMissingObject); //assign ROS topic msg to struct
    }
    //finished adding missing object data to pos 0 in 2d array

    //recalculate influence weighting for training session
    calculateInfluenceWeight();

    //start off with first object missing in sequence
    if (totalObjectsMissingStruct[detPos+1] == 0) { //if number of objects in next element is 0, no history exists
        //no history to compare with, therefore do all the calculation stuff
        if (DEBUG_missingObjectCallback) {
            cout << "nothing in struct pos " << detPos+1 << endl;
        }
        //first work out whether to add or delete from existing weighting value
        //no previous history, so add to weighting
        contextMissingNoHistory(detPos);
    }
    else {
        //history exists, therefore compare with history to see if object was in previous frame
        if (DEBUG_missingObjectCallback) {
            cout << "data exists in struct pos " << detPos+1 << endl;
        }
        contextMissingWithHistory();
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

    wheelchair_experiments_loc = tofToolBox->doesPkgExist("wheelchair_experiments");//check to see if dump package exists
    std::string PARAM_dataset_name;
    if (n.getParam("/wheelchair_robot/context/track_name", PARAM_dataset_name)) {
        ROS_INFO("Got param: %s", PARAM_dataset_name.c_str());
        experiments_loc_file = wheelchair_experiments_loc + experiments_loc + PARAM_dataset_name + ".txt";
        cout << "experiments file is located at " << experiments_loc_file << endl;
        trackingFileToArray();
        populateObjectsToTrack();
        objectsToTrack = 1;
    }
    else {
        ROS_ERROR("Failed to get param '/wheelchair_robot/context/track_name'");
        objectsToTrack = 0;
    }

    //get param to add duration from previous run
    if (n.getParam("/wheelchair_robot/context/add_to_duration", PARAM_add_to_duration)) {
        ROS_INFO("Got param: %s", PARAM_dataset_name.c_str());
        cout << "add_to_duration is " << PARAM_add_to_duration << endl;
    }
    else {
        ROS_ERROR("Failed to get param, add_to_duration remains at 0.0");
        PARAM_add_to_duration = 0.0;
    }

    ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 1000, objectLocationsCallback); //full list of objects

    //delay object detected thread by a few milliseconds, to allow the full objects list to be processed
    ros::NodeHandle n_delayThread;
    ros::CallbackQueue callback_queue_delayThread;
    n_delayThread.setCallbackQueue(&callback_queue_delayThread);
    ros::Subscriber detected_objects_sub = n_delayThread.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000, detectedObjectCallback); //detected objects in frame
    std::thread spinner_thread_delay([&callback_queue_delayThread]() {
        ros::SingleThreadedSpinner spinner_delay;
        spinner_delay.spin(&callback_queue_delayThread);
    });

    //delay object missing thread by a few milliseconds, to allow the full objects list to be processed
    ros::NodeHandle n_missingObjectsThread;
    ros::CallbackQueue callback_queue_missingObjectsThread;
    n_missingObjectsThread.setCallbackQueue(&callback_queue_missingObjectsThread);
    ros::Subscriber missing_objects_sub = n_missingObjectsThread.subscribe("wheelchair_robot/dacop/missing_objects/missing", 1000, missingObjectCallback); //detected objects in frame
    std::thread spinner_thread_missingObjects([&callback_queue_missingObjectsThread]() {
        ros::SingleThreadedSpinner spinner_missingObjects;
        spinner_missingObjects.spin(&callback_queue_missingObjectsThread);
    });

    ros::Publisher object_context_pub = n.advertise<wheelchair_msgs::objectContext>("/wheelchair_robot/context/objects", 1000); //publish object context info for decision making
    ptr_object_context = &object_context_pub; //pointer to publish object context

    //publish tracked objects context info
    ros::Publisher tracking_object_pub = n.advertise<wheelchair_msgs::trackingContext>("/wheelchair_robot/context/tracking", 1000);
    ptr_tracking_context = &tracking_object_pub;

    ros::Rate rate(10.0);


    while(ros::ok()) {
        if (DEBUG_main) {
            cout << "spin \n";
            for (int i = 0; i < totalObjectContextStruct; i++) { //print out context struct for debugging
                cout << objectContext[i].object_id << ":" << objectContext[i].object_name << endl;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    //objectNotDetected(); //call function to reduce context score
    if (saveDataToList) {
        contextInfoToList(); //save training info
        contextStructToList(); //save object context
    }
    return 0;
}