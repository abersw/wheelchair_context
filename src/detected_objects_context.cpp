/*
 * detected_objects_context.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "wheelchair_msgs/objectLocations.h"
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

const int DEBUG_main = 0;

std::string userRoomName;

void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {

}

int main (int argc, char **argv) {
    //add code here
    //notes:
    //take UID from publish_objects_location and pass it through here
    //when msg comes through with UID of object - append a room name to the object
    ros::init(argc, argv, "detected_objects_context");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 10, objectLocationsCallback);
    
    ros::Rate rate(10.0);
    while(ros::ok()) {
        n.getParam("/wheelchair_robot/param/user/room_name", userRoomName);
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}