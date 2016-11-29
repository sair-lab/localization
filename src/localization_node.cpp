// Copyright (c) <2016>, <Nanyang Technological University> All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <ros/ros.h>
#include "localization.h"
#include <geometry_msgs/Pose.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace message_filters;

int main(int argc, char** argv) 
{   

    ros::init(argc, argv, "ni_slam_node");

    ros::NodeHandle n("~");

    Localization localization(5);

    ros::Subscriber pose_sub = n.subscribe("incremental_pose_cov", 10, &Localization::addPoseEdge, &localization); 

    ros::Subscriber range_sub = n.subscribe("range", 10, &Localization::addRangeEdge, &localization);
     
    // where to add Imu package for getting the topic "/imu/" ?

    ros::NodeHandle nh;

    message_filters::Subscriber<uwb_driver::UwbRange> uwb_sub(nh, "range", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "imu", 1);

    typedef sync_policies::ApproximateTime<uwb_driver::UwbRange, sensor_msgs::Imu> imu_uwbSyncPolicy;

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

    Synchronizer<imu_uwbSyncPolicy> sync(imu_uwbSyncPolicy(10), uwb_sub, imu_sub);

    sync.registerCallback(boost::bind(&Localization::IMU_UWB, &localization, _1, _2));

    ros::spin();

    return 0;



}
