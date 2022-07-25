/*
 * tracking_context.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Alpha
 * 
*/

#include "tof_tool/tof_tool_box.h"

#include "wheelchair_msgs/trackingContext.h"


using namespace std;

static const int DEBUG_main = 0;
static const int DEBUG_fileLocations = 1;
static const int DEBUG_printTrackingMsg = 1;
static const int DEBUG_trackingCallback = 1;

//context data to save
struct TrackingObjects {
    double object_timestamp; //should be saved in seconds .toSec()
    int object_id;
    string object_name;
    float object_confidence; //object confidence from dnn
    int object_detected; //times object has been detected

    double object_weighting; //object weighting result
    double object_uniqueness; //object uniqueness result
    double object_score; //calculation of object weighting and uniqueness
    int object_instances; //number of objects in env

    int detected_or_missing; //flag if object was detected or expected but missing
};
static const int memTotalObjectsTracked = 100;
static const long memTotalObjectsTrackedCaptured = 10000;
//[0] contains object id and name [1] instances detected
struct TrackingObjects trackingObjects[memTotalObjectsTracked][memTotalObjectsTrackedCaptured];

//contains instances of object found, uses order from trackingObjects
int totalTrackingObjectsCaptured[memTotalObjectsTracked];
static int totalObjectsToTrack = 0;

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

//list of file locations
std::string wheelchair_dump_loc; //location of wheelchair_dump package
const static std::string dump_experiments_loc = "/dump/experiments/"; //location of experiments dir in wheelchair_dump

TofToolBox *tofToolBox;

void printTrackingMsg(const wheelchair_msgs::trackingContext objTrack) {
    tofToolBox->printSeparator(0);
    cout.precision(12);
    cout << fixed << objTrack.object_timestamp <<" : ";
    cout << objTrack.object_id << " : " <<
            objTrack.object_name << " : " <<
            "c: " << objTrack.object_confidence << " : " <<
            "d: " << objTrack.object_detected << " : " <<
            "w: " << objTrack.object_weighting << " : " <<
            "u: " << objTrack.object_uniqueness << " : " <<
            "s: " << objTrack.object_score << " : " <<
            "i: " << objTrack.object_instances << " : " <<
            objTrack.detected_or_missing << endl;
}

/**
 * Main callback function triggered by tracking objects detected or expected but missing
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void trackingCallback(const wheelchair_msgs::trackingContext objTrack) {
    if (DEBUG_printTrackingMsg) {
        printTrackingMsg(objTrack);
    }
    int currentObjID = objTrack.object_id;
    std::string currentObjName = objTrack.object_name;

    //if no objects have been tracked, at first element to array
    if (totalObjectsToTrack == 0) {
        if (DEBUG_trackingCallback) {
            cout << "starting tracking, add to first element in struct" << endl;
        }
        trackingObjects[0][0].object_timestamp = objTrack.object_timestamp;
        trackingObjects[0][0].object_id = objTrack.object_id;
        trackingObjects[0][0].object_name = objTrack.object_name;
        trackingObjects[0][0].object_confidence = objTrack.object_confidence;
        trackingObjects[0][0].object_detected = objTrack.object_detected;

        trackingObjects[0][0].object_weighting = objTrack.object_weighting;
        trackingObjects[0][0].object_uniqueness = objTrack.object_uniqueness;
        trackingObjects[0][0].object_score = objTrack.object_score;
        trackingObjects[0][0].object_instances = objTrack.object_instances;

        totalTrackingObjectsCaptured[0] = 1; //add to number of times object has been tracked
        totalObjectsToTrack++; //add to number of individual objects to track
    }
    //if objects have already been tracked
    else {
        //run through list of individual tracked objects
        int trackedObjListPos = -1;
        for (int isTrackedObjList = 0; isTrackedObjList < totalObjectsToTrack; isTrackedObjList++) {
            int currentTrackedObjID = trackingObjects[isTrackedObjList][0].object_id;
            std::string currentTrackedObjName = trackingObjects[isTrackedObjList][0].object_name;
            //if ros msg object is the same as tracked object list
            if ((currentObjID == currentTrackedObjID) && (currentObjName == currentTrackedObjName)) {
                trackedObjListPos = isTrackedObjList; //retrieve position of object in tracking struct
            }
        }
        if (trackedObjListPos != -1) {
            //found object in tracking list, add to end of array
            if (DEBUG_trackingCallback) {
                cout << "found object in tracking struct, adding to next position in struct";
            }
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_timestamp = objTrack.object_timestamp;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_id = objTrack.object_id;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_name = objTrack.object_name;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_confidence = objTrack.object_confidence;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_detected = objTrack.object_detected;

            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_weighting = objTrack.object_weighting;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_uniqueness = objTrack.object_uniqueness;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_score = objTrack.object_score;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_instances = objTrack.object_instances;

            totalTrackingObjectsCaptured[trackedObjListPos]++; //add to number of times object has been tracked
        }
        else {
            //object not found in tracking list, add object to list
            //set index to 0 on new tracking object
            if (DEBUG_trackingCallback) {
                cout << "object not detected in tracking struct, adding new object" << endl;
            }
            totalTrackingObjectsCaptured[totalObjectsToTrack] = 0;
            trackedObjListPos = totalObjectsToTrack; //set tracked object positon to next element in array

            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_timestamp = objTrack.object_timestamp;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_id = objTrack.object_id;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_name = objTrack.object_name;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_confidence = objTrack.object_confidence;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_detected = objTrack.object_detected;

            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_weighting = objTrack.object_weighting;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_uniqueness = objTrack.object_uniqueness;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_score = objTrack.object_score;
            trackingObjects[trackedObjListPos][totalTrackingObjectsCaptured[trackedObjListPos]].object_instances = objTrack.object_instances;

            totalTrackingObjectsCaptured[trackedObjListPos] = 1; //add to number of times object has been tracked
            totalObjectsToTrack++; //add to number of individual objects to track
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
    ros::init(argc, argv, "tracking_context");
    ros::NodeHandle n;

    wheelchair_dump_loc = tofToolBox->doesPkgExist("wheelchair_dump");//check to see if dump package exists
    std::string wheelchair_experiments_loc = wheelchair_dump_loc + dump_experiments_loc;

    ros::Subscriber tracking_sub = n.subscribe("/wheelchair_robot/context/tracking", 1000, trackingCallback); //tracked object


    ros::Rate rate(10.0);
    while(ros::ok()) {
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    //print tracking struct
    cout << "saving context struct" << endl;
    for (int isTrackingList = 0; isTrackingList < totalObjectsToTrack; isTrackingList++) {
        tofToolBox->printSeparator(0);
        for (int isTrackingObject = 0; isTrackingObject < totalTrackingObjectsCaptured[isTrackingList]; isTrackingObject++) {
            cout.precision(12);
            cout << fixed << trackingObjects[isTrackingList][isTrackingObject].object_timestamp << " : ";
            cout << trackingObjects[isTrackingList][isTrackingObject].object_id << " : " <<
                    trackingObjects[isTrackingList][isTrackingObject].object_name << " : " <<
                    trackingObjects[isTrackingList][isTrackingObject].object_confidence << " : " <<
                    trackingObjects[isTrackingList][isTrackingObject].object_detected << " : " <<
                    trackingObjects[isTrackingList][isTrackingObject].object_weighting << " : " <<
                    trackingObjects[isTrackingList][isTrackingObject].object_uniqueness << " : " <<
                    trackingObjects[isTrackingList][isTrackingObject].object_score << " : " <<
                    trackingObjects[isTrackingList][isTrackingObject].object_instances << " : " <<
                    trackingObjects[isTrackingList][isTrackingObject].detected_or_missing << endl;
        }
    }
    return 0;
}