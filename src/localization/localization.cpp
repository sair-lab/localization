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

    solver = new Solver();

    solver->setBlockOrdering(false);

    se3blockSolver = new SE3BlockSolver(solver);

    optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);

    optimizer.setAlgorithm(optimizationsolver);

    optimizer.setVerbose(false);

    start_time = 0;

    uwb_number = 0;

    judge = 0;

    last_last_vertex->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,0)));

    iteration_max = 20;

    robot_max_velocity = 2.0;

    self_id = nodesId.back();

    robots.emplace(self_id, Robot(self_id%100, false, optimizer));
    ROS_WARN("Init self robot ID: %d with moving option", self_id);

    for (size_t i = 0; i < nodesId.size()-1; ++i)
    {
        robots.emplace(nodesId[i], Robot(nodesId[i]%100, true));
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose(0,3) = nodesPos[i*3];
        pose(1,3) = nodesPos[i*3+1];
        pose(2,3) = nodesPos[i*3+2];
        robots.at(nodesId[i]).init(optimizer, pose);
        ROS_WARN("Init robot ID: %d with position (%.2f,%.2f,%.2f)", nodesId[i], pose(0,3), pose(1,3), pose(2,3));
    }
}


void Localization::solve()
{
    optimizer.initializeOptimization();

    optimizer.optimize(iteration_max);

    auto vertex = robots.at(self_id).last_vertex()->estimate();

    geometry_msgs::PoseStamped pose;

    pose.header = robots.at(self_id).last_header();

    pose.header.frame_id = "world";

    tf::poseEigenToMsg(vertex, pose.pose);

    pose_pub.publish(pose);

    // ROS_INFO("Localization: graph optimized!");

    // optimizer.save( "result.g2o" );

}


void Localization::addPoseEdge(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_cov_)
{
    geometry_msgs::PoseWithCovarianceStamped pose_cov(*pose_cov_);

    auto last_vertex = robots.at(self_id).last_vertex(sensor_type.pose);

    g2o::VertexSE3* new_vertex;

    if (pose_cov.header.frame_id != robots.at(self_id).last_header(sensor_type.pose).frame_id)
        new_vertex = robots.at(self_id).new_vertex(sensor_type.pose, pose_cov.header, optimizer);
    else
        new_vertex = robots.at(self_id).new_vertex(sensor_type.general, pose_cov.header, optimizer);

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->vertices()[0] = last_vertex;

    edge->vertices()[1] = new_vertex;

    Eigen::Isometry3d measurement;

    tf::poseMsgToEigen(pose_cov.pose.pose, measurement);

    edge->setMeasurement(measurement);

    Eigen::Map<Eigen::MatrixXd> covariance(pose_cov.pose.covariance.data(), 6, 6);

    edge->setInformation(covariance.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    optimizer.addEdge(edge);

    ROS_WARN("Localization: added pose edge id: %d frame_id: %s", pose_cov.header.seq, pose_cov.header.frame_id.c_str());
}


void Localization::addRangeEdge(const uwb_driver::UwbRange::ConstPtr& uwb)
{
    // if (!uwb_number)
    // {
    //     double start_time = uwb->header.stamp.toSec();
    //     uwb_number =1;
    // }

    double dt_requester = uwb->header.stamp.toSec() - robots.at(uwb->requester_id).last_header().stamp.toSec();
    double dt_responder = uwb->header.stamp.toSec() - robots.at(uwb->responder_id).last_header().stamp.toSec();

    cout << "interval time" << uwb->header.stamp.toSec()-start_time <<endl;

    auto vertex_last_requester = robots.at(uwb->requester_id).last_vertex();
    auto vertex_last_responder = robots.at(uwb->responder_id).last_vertex();

    // requester to responder edge
    auto vertex_requester = robots.at(uwb->requester_id).new_vertex(sensor_type.range, uwb->header, optimizer);
    auto vertex_responder = robots.at(uwb->responder_id).new_vertex(sensor_type.range, uwb->header, optimizer);
    auto edge = create_range_edge(vertex_requester, vertex_responder, uwb->distance, pow(uwb->distance_err, 2));
    optimizer.addEdge(edge);

    if (!robots.at(uwb->requester_id).is_static())
    {
        ROS_WARN("adding requester trajectory edge");
        auto edge_requester_range = create_range_edge(vertex_last_requester, vertex_requester, 0, robot_max_velocity*robot_max_velocity*dt_requester*dt_requester);
        optimizer.addEdge(edge_requester_range);
    }

    if (!robots.at(uwb->responder_id).is_static())
    {
        ROS_WARN("adding responder trajectory edge");
        auto edge_responder_range = create_range_edge(vertex_last_responder, vertex_responder, 0, robot_max_velocity*robot_max_velocity*dt_responder*dt_responder);
        optimizer.addEdge(edge_responder_range);
    }

    ROS_WARN("Localization: added range edge id: %d", uwb->header.seq);

    solve();

    timer.toc();
}


void Localization::addTwistEdge(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twist_)
{
    geometry_msgs::TwistWithCovarianceStamped twist(*twist_);

    double dt = twist.header.stamp.toSec() - robots.at(self_id).last_header().stamp.toSec();

    auto last_vertex = robots.at(self_id).last_vertex();

    auto new_vertex = robots.at(self_id).new_vertex(sensor_type.twist, twist.header, optimizer);

    auto edge = create_se3_edge_from_twist(last_vertex, new_vertex, twist.twist, dt);

    optimizer.addEdge(edge);

    ROS_WARN("Localization: added twist edge id: %d!", twist.header.seq);
}


void Localization::addImuEdge(const uwb_driver::UwbRange::ConstPtr& uwb,const sensor_msgs::Imu::ConstPtr& Imu_)
{

    
    double dt_requester = uwb->header.stamp.toSec() - robots.at(uwb->requester_id).last_header(sensor_type.range).stamp.toSec();
    // double dt_responder = uwb->header.stamp.toSec() - robots.at(uwb->responder_id).last_header().stamp.toSec();

    auto vertex_last_requester = robots.at(uwb->requester_id).last_vertex(sensor_type.range);
    uwb_number = uwb_number + 1;
    // auto vertex_last_responder = robots.at(uwb->responder_id).last_vertex();

    // requester to responder edge
    auto vertex_requester = robots.at(uwb->requester_id).new_vertex(sensor_type.range, uwb->header, optimizer);
    auto vertex_responder = robots.at(uwb->responder_id).new_vertex(sensor_type.range, uwb->header, optimizer);
    auto edge = create_range_edge(vertex_requester, vertex_responder, uwb->distance, pow(uwb->distance_err, 2));
    optimizer.addEdge(edge);

    bool requester_same_frame_id = (robots.at(uwb->requester_id).last_header().frame_id == uwb->header.frame_id);
    bool requester_not_static = robots.at(uwb->requester_id).not_static();


    sensor_msgs::Imu Imu(*Imu_);
    // calculate translation
    double qx = Imu.orientation.x;
    double qy = Imu.orientation.y;
    double qz = Imu.orientation.z;
    double qw = Imu.orientation.w;

    Matrix3d T_matrix;
    T_matrix << 1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw,
                2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                2*qx*qz-2*qy*qw, 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy;

    Vector3d imu_acceleration = Vector3d(Imu.linear_acceleration.x,Imu.linear_acceleration.y,Imu.linear_acceleration.z);
    Vector3d gravity = Vector3d(0,0,9.8);
    Vector3d acceleration = T_matrix.inverse()*imu_acceleration+gravity;

    Vector3d  last_vertex_velocity;
    if(uwb_number < 2)
    {
        last_vertex_velocity = Vector3d::Zero();
    }
    else
    {
        last_vertex_velocity = vertex_last_requester->estimate().translation() - last_last_vertex->estimate().translation();
        judge = last_vertex_velocity.norm();
        cout << "judge" << '\n' <<  judge <<endl;

        last_vertex_velocity = last_vertex_velocity/dt_requester;

        // last_vertex_velocity = Vector3d::Identity()*2;

        // if (last_vertex_velocity.norm()>=2*sqrt(3))
        // {
        //     last_vertex_velocity << 2,2,2;  
        // }
    }

    last_last_vertex = vertex_last_requester;


    // cout << "matrix" << '\n' << last_vertex_velocity <<endl;

    if (uwb_number >20)
    {
    Vector3d  translation = last_vertex_velocity*dt_requester+0.5*acceleration*pow(dt_requester,2);

    // cout << "translation" << '\n' <<  translation <<endl;

    // cout << "translation1" << '\n' <<  acceleration <<endl;


    Eigen::Isometry3d current_pose;
    current_pose.setIdentity(); // very important

    Quaterniond imu_rotation = Quaterniond(Imu.orientation.w,Imu.orientation.x,Imu.orientation.y,Imu.orientation.z);

    current_pose.rotate(imu_rotation);
    current_pose.translate(g2o::Vector3D(1, 1, 1));

    // cout << "information" << '\n' << current_pose.matrix() << endl;

    // cout << "information" << '\n' << current_pose.rotation() << endl;

    // cout << "information" << '\n' << current_pose.translation() << endl;

    // calculate transform matrix
    Isometry3d transform_matrix = vertex_last_requester->estimate().inverse()*current_pose;
    translation = current_pose.rotation().inverse()*translation;
    transform_matrix(0,3) = translation[0];
    transform_matrix(1,3) = translation[1];
    transform_matrix(2,3) = translation[2];

    // cout << "information" << '\n' <<  transform_matrix.matrix() <<endl;

    Eigen::Map<Eigen::MatrixXd> covariance_orientation(Imu.orientation_covariance.data(), 3, 3);
    Eigen::Map<Eigen::MatrixXd> covariance_translation(Imu.linear_acceleration_covariance.data(), 3, 3);   
    Eigen::Matrix3d zero_matrix = Eigen::Matrix3d::Zero();

    Eigen::MatrixXd SE3information(6,6);

    // covariance_orientation = covariance_orientation*pow(dt_requester,2);

    covariance_translation << robot_max_velocity*dt_requester,0,0,
                              0,robot_max_velocity*dt_requester,0,
                              0,0,robot_max_velocity*dt_requester;

    SE3information << covariance_orientation,zero_matrix,
                      zero_matrix,covariance_translation;

    // cout << "orientation" << '\n' <<  Imu.orientation_covariance[8] <<endl;

    // cout << "orientation" << '\n' <<  covariance_orientation <<endl;

    // cout << "information" << '\n' <<  SE3information <<endl;

    // cout << "information inverse" << '\n' <<  SE3information.inverse().matrix() << endl;


    // requester to requester's last vertex edge

    if (requester_not_static)
    {
        ROS_WARN("IMU IMU IMU");

        g2o::EdgeSE3 *SE3edge = new g2o::EdgeSE3();
    
        SE3edge->vertices()[0] = vertex_last_requester;

        SE3edge->vertices()[1] = vertex_requester; 

        SE3edge->setMeasurement(transform_matrix);

        SE3edge->setInformation(SE3information);

        edge->setRobustKernel(new g2o::RobustKernelHuber());
    
        optimizer.addEdge( SE3edge );

        
    }

    // if (!robots.at(uwb->requester_id).is_static())
    // {
    //     ROS_WARN("adding requester trajectory edge");
    //     auto edge_requester_range = create_range_edge(vertex_last_requester, vertex_requester, translation.norm(),robot_max_velocity*robot_max_velocity*dt_requester*dt_requester);
    //     optimizer.addEdge(edge_requester_range);

    //     cout << "norm" << translation.norm() << endl;
    // }

    }

    ROS_WARN("Localization: added range edge id: %d", uwb->header.seq);

    solve();

    timer.toc();
}


inline Eigen::Isometry3d Localization::twist2transform(geometry_msgs::TwistWithCovariance& twist, Eigen::ArrayXXd& covariance, double dt)
{
    tf::Vector3 translation, euler;

    tf::vector3MsgToTF(twist.twist.linear, translation);

    tf::vector3MsgToTF(twist.twist.angular, euler);

    tf::Quaternion quaternion;

    quaternion.setRPY(euler[0]*dt, euler[1]*dt, euler[2]*dt);

    tf::Transform transform(quaternion, translation * dt);

    Eigen::Isometry3d measurement;

    tf::transformTFToEigen(transform, measurement);

    Eigen::Map<Eigen::ArrayXXd> cov(twist.covariance.data(), 6, 6);

    covariance = cov*dt*dt;

    return measurement;
}


inline g2o::EdgeSE3* Localization::create_se3_edge_from_twist(g2o::VertexSE3* vetex1, g2o::VertexSE3* vetex2, geometry_msgs::TwistWithCovariance& twist, double dt)
{
    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->vertices()[0] = vetex1;

    edge->vertices()[1] = vetex2;

    Eigen::ArrayXXd covariance;

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

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    return edge;
}
