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
static const int DEBUG_trackingCallback = 0;

//context data to save
struct TrackingObjects {
    double object_timestamp; //should be saved in seconds .toSec()
    int object_id;
    string object_name;
    float object_confidence; //object confidence from dnn

    double object_weighting; //object weighting result
    double object_uniqueness; //object uniqueness result
    double object_score; //calculation of object weighting and uniqueness
    int object_instances; //number of objects in env
};
struct TrackingObjects trackingObjects[10000];

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
    cout << objTrack.object_timestamp << " : " <<
            objTrack.object_id << " : " <<
            objTrack.object_name << " : " <<
            objTrack.object_confidence << " : " <<
            objTrack.object_detected << " : " <<
            objTrack.object_weighting << " : " <<
            objTrack.object_uniqueness << " : " <<
            objTrack.object_score << " : " <<
            objTrack.object_instances << " : " <<
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

    return 0;
}