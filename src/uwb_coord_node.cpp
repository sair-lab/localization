#include <ros/ros.h>
#include "uwb.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwb_coord");

    ros::NodeHandle n("~");

    UWB uwb(n); //created a uwb instance 
    
    string uwb_coord_topic;

    ros::Subscriber uwb_coord_sub;
    
    if(n.getParam("topic/uwb_coord", uwb_coord_topic)) //if the param for topic is not null
    {   

        uwb.addPriorEdge(4); //parameter is the total number of anchor uwb
        
        uwb_coord_sub = n.subscribe(uwb_coord_topic, 1, &UWB::addRangeEdge, &uwb);        
        
        ROS_WARN("Subscribing to: %s", uwb_coord_topic.c_str());    

        uwb.publish();  
        
        uwb.solve(); //put solve outside loop, only solve after adding all edges
        
        uwb.save();
        
        ROS_WARN("Optimized file saved");    
    }

    ros::spin();

    return 0;
}