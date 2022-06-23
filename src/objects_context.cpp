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


static const int DEBUG_main = 0;

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
    ros::init(argc, argv, "objects_context");
    ros::NodeHandle n;




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