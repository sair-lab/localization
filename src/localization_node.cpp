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

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace message_filters;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ni_slam_node");

    ros::NodeHandle n("~");

    Localization localization(n);

    string pose_topic, range_topic, lidar_topic, imu_topic, twist_topic, relative_topic;

    ros::Subscriber pose_sub, range_sub, imu_sub, twist_sub, relative_sub;


    if(n.getParam("topic/pose", pose_topic))
    {
        pose_sub = n.subscribe(pose_topic, 1000, &Localization::addPoseEdge, &localization);
        ROS_WARN("Subscribing to: %s",pose_topic.c_str());
    }

    if(n.getParam("topic/range", range_topic))
    {
        range_sub = n.subscribe(range_topic, 1, &Localization::addRangeEdge, &localization);
        ROS_WARN("Subscribing to: %s", range_topic.c_str());
    }

    if(n.getParam("topic/twist", twist_topic))
    {
        twist_sub = n.subscribe(twist_topic, 1, &Localization::addTwistEdge, &localization);
        ROS_WARN("Subscribing to: %s", twist_topic.c_str());
    }

    if(n.getParam("topic/lidar", lidar_topic))
    {
        twist_sub = n.subscribe(lidar_topic, 1, &Localization::addLidarEdge, &localization);
        ROS_WARN("Subscribing to: %s", lidar_topic.c_str());
    }

    if(n.getParam("topic/imu", imu_topic))
    {
        imu_sub = n.subscribe(imu_topic, 1, &Localization::addImuEdge, &localization);
        ROS_WARN("Subscribing to: %s", imu_topic.c_str());
    }

#ifdef RELATIVE_LOCALIZATION
    if(n.getParam("topic/relative_range", relative_topic))
    {
        relative_sub = n.subscribe(relative_topic, 1, &Localization::addRLRangeEdge, &localization);
        ROS_WARN("Subscribing to: %s", relative_topic.c_str());
    }
#endif

    dynamic_reconfigure::Server<localization::localizationConfig> dr_srv;

    dynamic_reconfigure::Server<localization::localizationConfig>::CallbackType cb;

    cb = boost::bind(&Localization::configCallback, &localization, _1, _2);

    dr_srv.setCallback(cb);

    ros::spin();

    return 0;
}
