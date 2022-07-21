/*
 * tracking_context.cpp
 * wheelchair_context
 * version: 0.1.0 Majestic Maidenhair
 * Status: Beta
 * 
*/

#include "tof_tool/tof_tool_box.h"

#include "wheelchair_msgs/trackingContext.h"


using namespace std;

static const int DEBUG_main = 0;
static const int DEBUG_fileLocations = 1;

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

//list of file locations
std::string wheelchair_dump_loc; //location of wheelchair_dump package
const static std::string dump_experiments_loc = "/dump/experiments/"; //location of experiments dir in wheelchair_dump

TofToolBox *tofToolBox;

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

    //ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 1000, objectLocationsCallback); //full list of objects


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