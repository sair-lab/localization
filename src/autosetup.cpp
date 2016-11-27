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
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

using namespace std; 

int main(int argc, char** argv) 
{   

    ros::init(argc, argv, "autosetup"); 

    ros::NodeHandle n("~");

    ros::Publisher initialposition = n.advertise<geometry_msgs::PoseStamped> ("initialposition", 5);

    Localization localization(5);

    ros::Time starttime = ros::Time::now();
 
    geometry_msgs::PoseStamped::ConstPtr position1;
    geometry_msgs::PoseStamped position2;

    while (ros::ok)

    {
        // need to design a method to fill the linkmatrix

     ros::Subscriber range_sub = n.subscribe("range", 10, &Localization::setup, &localization);


         if ((ros::Time::now()- starttime).toSec()>2) 

        {      
               position2 = localization.beginsolve(position1);

               initialposition.publish(position2);

        }

    }
        
    ros::spin();
    return 0;


}
