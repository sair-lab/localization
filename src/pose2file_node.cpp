#include <ros/ros.h>
#include "localization.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
using namespace std;

std::ofstream file1;
std::ofstream file2;
std::ofstream file3;

 void estimate_record(const geometry_msgs::PoseStamped::ConstPtr& pose_)
{
    geometry_msgs::PoseStamped pose(*pose_);

    file1.open("/home/xufang/experiment_data/estimate_data.txt",ios::app);

    file1<< pose.header.stamp <<" "<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z <<" "\
        <<pose.pose.orientation.x<<" "<< pose.pose.orientation.y<<" "<<pose.pose.orientation.z<<" "<<pose.pose.orientation.w\
        <<endl;

    file1.close();  

   // ROS_WARN("HERE!!");

}

void truth_record(const geometry_msgs::PoseStamped::ConstPtr& pose_)
{

    // ROS_WARN("WE ARE HERE!!");
    geometry_msgs::PoseStamped pose(*pose_);

    file2.open("/home/xufang/experiment_data/truth_data.txt",ios::app); 

    file2<< pose.header.stamp <<" "<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z <<" "\
        <<pose.pose.orientation.x<<" "<< pose.pose.orientation.y<<" " << pose.pose.orientation.z<<" "<<pose.pose.orientation.w\
        <<endl; 
    file2.close();
}



void imu_record(const sensor_msgs::Imu::ConstPtr& pose_)
{

    // ROS_WARN("WE ARE HERE!!");
    sensor_msgs::Imu pose(*pose_);

    file3.open("/home/xufang/experiment_data/imu_data.txt",ios::app); 

    file3<< pose.header.stamp <<" "<<pose.orientation.w<<" "<<pose.orientation.x<<" "<<pose.orientation.y <<" "\
        <<pose.orientation.z <<endl; 
    file3.close();
}



int main(int argc, char** argv)
{
   
    ros::init(argc, argv, "error_node");

    ros::NodeHandle n;

    file1.open("/home/xufang/experiment_data/estimate_data.txt", ios::trunc|ios::out);
              
    file1.close();

    file2.open("/home/xufang/experiment_data/truth_data.txt", ios::trunc|ios::out);
              
    file2.close();

    file3.open("/home/xufang/experiment_data/imu_data.txt", ios::trunc|ios::out);
              
    file3.close();



    ros::Subscriber estimate_sub = n.subscribe("/localization_node/optimized/pose", 100, estimate_record);

    // ros::Subscriber true_sub = n.subscribe("/vicon_xb_node/mocap/pose", 100, truth_record);

    ros::Subscriber true_sub = n.subscribe("/viconxbee_node/mocap/pose", 100, truth_record);

    ros::Subscriber imu_sub = n.subscribe("/imu/data", 100, imu_record);

    ros::spin();

    return 0;
}



 



