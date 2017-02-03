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

#include "localization.h"

Localization::Localization(ros::NodeHandle n, std::vector<int> nodesId, std::vector<double> nodesPos)
{
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("optimized/pose", 1);

    path_pub = n.advertise<nav_msgs::Path>("optimized/path", 1);

    solver = new Solver();

    solver->setBlockOrdering(false);

    se3blockSolver = new SE3BlockSolver(solver);

    optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);

    optimizer.setAlgorithm(optimizationsolver);

    optimizer.setVerbose(false);

    iteration_max = 20;

    robot_max_velocity = 1;

    self_id = nodesId.back();

    robots.emplace(self_id, Robot(self_id, false, optimizer));
    ROS_INFO("Init self robot ID: %d with moving option", self_id);

    for (size_t i = 0; i < nodesId.size()-1; ++i)
    {
        robots.emplace(nodesId[i], Robot(nodesId[i], true));
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose(0,3) = nodesPos[i*3];
        pose(1,3) = nodesPos[i*3+1];
        pose(2,3) = nodesPos[i*3+2];
        robots.at(nodesId[i]).init(optimizer, pose);
        ROS_INFO("Init robot ID: %d with position (%.2f,%.2f,%.2f)", nodesId[i], pose(0,3), pose(1,3), pose(2,3));
    }
}


void Localization::solve()
{
    timer.tic();

    optimizer.initializeOptimization();

    optimizer.optimize(iteration_max);

    // optimizer.save("/home/eee/after.g2o");

    // auto edges = optimizer.activeEdges();
    // if(edges.size()>100)
    // {
    //     for(auto edge:edges)
    //         if (edge->chi2() > 2.0 && edge->dimension () ==1)
    //         {
    //             edge->setLevel(1);
    //             ROS_WARN("Removed one Range Edge");
    //         }
    // }
    // optimizer.optimize(iteration_max);

    ROS_INFO("Graph optimized with error: %f!", optimizer.chi2());

    timer.toc();
}


void Localization::publish()
{
    auto pose = robots.at(self_id).current_pose();

    pose_pub.publish(pose);

    if(flag_save_file)
        save_file(pose);

    path_pub.publish(*robots.at(self_id).vertices2path());
}


void Localization::addPoseEdge(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_cov_)
{
    geometry_msgs::PoseWithCovarianceStamped pose_cov(*pose_cov_);

    if (pose_cov.header.frame_id != robots.at(self_id).last_header(sensor_type.pose).frame_id)
        key_vertex = robots.at(self_id).last_vertex(sensor_type.pose);

    auto new_vertex = robots.at(self_id).new_vertex(sensor_type.pose, pose_cov.header, optimizer);

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->vertices()[0] = key_vertex;

    edge->vertices()[1] = new_vertex;

    Eigen::Isometry3d measurement;

    tf::poseMsgToEigen(pose_cov.pose.pose, measurement);

    edge->setMeasurement(measurement);

    Eigen::Map<Eigen::MatrixXd> covariance(pose_cov.pose.covariance.data(), 6, 6);

    edge->setInformation(covariance.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    optimizer.addEdge(edge);

    ROS_INFO("added pose edge id: %d frame_id: %s;", pose_cov.header.seq, pose_cov.header.frame_id.c_str());

    solve();

    publish();
}

int num_uwb=100;
int uwb_index=0;
unsigned char index_uwb[4]={100,101,102,103};

void Localization::addRangeEdge(const uwb_driver::UwbRange::ConstPtr& uwb)
{

    if(num_uwb > 2)
    {
        num_uwb++;
        if (uwb->responder_id != index_uwb[uwb_index])
            return;
        else
        {
            uwb_index=(uwb_index+1)%4;
            num_uwb=0;
        }
    }
    else
    {
        num_uwb++;
        return;
    }

    double dt_requester = uwb->header.stamp.toSec() - robots.at(uwb->requester_id).last_header().stamp.toSec();
    double dt_responder = uwb->header.stamp.toSec() - robots.at(uwb->responder_id).last_header().stamp.toSec();
    double distance_cov = pow(uwb->distance_err, 2);
    double cov_requester = pow(robot_max_velocity*dt_requester/3, 2); //3 sigma priciple

    auto vertex_last_requester = robots.at(uwb->requester_id).last_vertex();
    auto vertex_last_responder = robots.at(uwb->responder_id).last_vertex();
    auto vertex_responder = robots.at(uwb->responder_id).new_vertex(sensor_type.range, uwb->header, optimizer);

    auto frame_id = robots.at(uwb->requester_id).last_header().frame_id;

    if(frame_id == uwb->header.frame_id || frame_id == "none")
    {    
        auto vertex_requester = robots.at(uwb->requester_id).new_vertex(sensor_type.range, uwb->header, optimizer);

        auto edge = create_range_edge(vertex_requester, vertex_responder, uwb->distance, distance_cov);

        auto edge_requester_range = create_range_edge(vertex_last_requester, vertex_requester, 0, cov_requester);

        optimizer.addEdge(edge_requester_range);

        optimizer.addEdge(edge);

        ROS_INFO("added two requester range edge with id: <%d>;",uwb->responder_id);
    }
    else
    {
        auto edge = create_range_edge(vertex_last_requester, vertex_responder, uwb->distance, distance_cov + cov_requester);

        optimizer.addEdge(edge);

        ROS_INFO("added requester edge with id: <%d>", uwb->responder_id);
    }

    if (!robots.at(uwb->responder_id).is_static())
    {
        double cov_responder = pow(robot_max_velocity*dt_responder/3, 2); //3 sigma priciple

        auto edge_responder_range = create_range_edge(vertex_last_responder, vertex_responder, 0, cov_responder);

        optimizer.addEdge(edge_responder_range);

        ROS_INFO("added responder trajectory edge;");
    }

    solve();

    publish();
}


void Localization::addTwistEdge(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twist_)
{
    geometry_msgs::TwistWithCovarianceStamped twist(*twist_);

    double dt = twist.header.stamp.toSec() - robots.at(self_id).last_header().stamp.toSec();

    auto last_vertex = robots.at(self_id).last_vertex();

    auto new_vertex = robots.at(self_id).new_vertex(sensor_type.twist, twist.header, optimizer);

    auto edge = create_se3_edge_from_twist(last_vertex, new_vertex, twist.twist, dt);

    optimizer.addEdge(edge);

    ROS_INFO("Localization: added twist edge id: %d!", twist.header.seq);

    solve();
}


void Localization::addImuEdge(const sensor_msgs::Imu::ConstPtr& imu)
{

}


inline Eigen::Isometry3d Localization::twist2transform(geometry_msgs::TwistWithCovariance& twist, Eigen::MatrixXd& covariance, double dt)
{
    tf::Vector3 translation, euler;

    tf::vector3MsgToTF(twist.twist.linear, translation);

    tf::vector3MsgToTF(twist.twist.angular, euler);

    tf::Quaternion quaternion;

    quaternion.setRPY(euler[0]*dt, euler[1]*dt, euler[2]*dt);

    tf::Transform transform(quaternion, translation * dt);

    Eigen::Isometry3d measurement;

    tf::transformTFToEigen(transform, measurement);

    Eigen::Map<Eigen::MatrixXd> cov(twist.covariance.data(), 6, 6);

    covariance = cov*dt*dt;

    return measurement;
}


inline g2o::EdgeSE3* Localization::create_se3_edge_from_twist(g2o::VertexSE3* vetex1, g2o::VertexSE3* vetex2, geometry_msgs::TwistWithCovariance& twist, double dt)
{
    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->vertices()[0] = vetex1;

    edge->vertices()[1] = vetex2;

    Eigen::MatrixXd covariance;

    auto measurement = twist2transform(twist, covariance, dt);

    edge->setMeasurement(measurement);

    edge->setInformation(covariance.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    return edge;
}


inline g2o::EdgeSE3Range* Localization::create_range_edge(g2o::VertexSE3* vertex1, g2o::VertexSE3* vertex2, double distance, double covariance)
{
    auto edge = new g2o::EdgeSE3Range();

    edge->vertices()[0] = vertex1;

    edge->vertices()[1] = vertex2;

    edge->setMeasurement(distance);

    Eigen::MatrixXd covariance_matrix = Eigen::MatrixXd::Zero(1, 1);

    covariance_matrix(0,0) = covariance;

    edge->setInformation(covariance_matrix.inverse());

    edge->setRobustKernel(new g2o::RobustKernelPseudoHuber());

    return edge;
}


inline void Localization::save_file(geometry_msgs::PoseStamped pose)
{
    file.open(filename.c_str(), ios::app);

    file<<boost::format("%.9f") % (pose.header.stamp.toSec())<<" "
        <<pose.pose.position.x<<" "
        <<pose.pose.position.y<<" "
        <<pose.pose.position.z<<" "
        <<pose.pose.orientation.x<<" "
        <<pose.pose.orientation.y<<" "
        <<pose.pose.orientation.z<<" "
        <<pose.pose.orientation.w<<endl;
    
    file.close();
}


void Localization::set_file(string name_prefix)
{
    flag_save_file = true;
    char s[30];
    struct tm tim;
    time_t now;
    now = time(NULL);
    tim = *(localtime(&now));
    strftime(s,30,"_%Y_%b_%d_%H_%M_%S.txt",&tim);
    filename = name_prefix + string(s);
    file.open(filename.c_str(), ios::trunc|ios::out);
    file<<"# "<<"iteration_max:"<<iteration_max<<"\n";
    file.close();
}
