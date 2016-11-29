#include "ros/ros.h"
#include "std_msgs/String.h"
#include "check/Num.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string> 
#include <stdlib.h>

 
using namespace std;


int main(int argc, char **argv)

{
  
    ros::init(argc, argv, "talk");

    ros::NodeHandle n;

 

    tf::TransformBroadcaster br;

    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/path_pose", 20);

    ros::Rate loop_rate(50);

    geometry_msgs::PoseStamped position;

    tf::Transform transform;

    int counter = 0;

   
   while (ros::ok())

   {  
      int n=1;
      FILE * fp ;

    // please revise the file path
    fp = fopen ("/home/xufang/rosworkspace/src/data/src/data.txt", "r");


    while(n==1)
    {
        float x, y, z;
        int num = fscanf(fp, "%f,%f,%f\n", &x, &y, &z);
        position.pose.position.x = x;
        position.pose.position.y = y;
        position.pose.position.z = z;
        position.header.stamp=ros::Time::now();
        position.header.seq = counter ++;
        position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

        transform.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q = tf::createQuaternionFromRPY(0,0,0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "target", "local_origin"));
 
        if (num == 3)
           {
            chatter_pub.publish(position);
            ROS_INFO("ID: %d published <%f, %f, %f>", counter, position.pose.position.x, position.pose.position.y, position.pose.position.z);
           }

        else 
           { n=0;

           }
        // else
        // {
        //   ROS_WARN("FINISHED READING FILES OR FORMAT IS NOT CAMPITIBALE");
        //   exit(0);
        // }
        loop_rate.sleep(); 
    }

   }

  return 0;
}



