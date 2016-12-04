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
using namespace Eigen;

Localization::Localization() 
{
    solver = new Solver();

    solver->setBlockOrdering(false);

    se3blockSolver = new SE3BlockSolver(solver);

    optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);

    optimizer.setAlgorithm(optimizationsolver);

    optimizer.setVerbose(true);

    uwb_number = 0; // number of self_id as requester

    last_last_vertex->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,0)));

    iteration_max = 100;

    self_id = 100;

    robots.emplace(self_id, Robot(0, false, optimizer));
}


void Localization::solve()
{
    optimizer.initializeOptimization();

    optimizer.optimize(iteration_max);

    ROS_INFO("graph optimized!");
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

    optimizer.addVertex(new_vertex);

    optimizer.addEdge(edge);

    ROS_WARN("added pose edge id: %d", pose_cov.header.seq);
}


void Localization::addRangeEdge(const uwb_driver::UwbRange::ConstPtr& uwb,const sensor_msgs::Imu::ConstPtr& Imu_)
{
    robots.emplace(uwb->requester_id, Robot(uwb->requester_idx, true, optimizer));

    robots.emplace(uwb->responder_id, Robot(uwb->responder_idx, true, optimizer));

    double dt = uwb->header.stamp.toSec() - robots.at(self_id).last_header(sensor_type.range).stamp.toSec();

    auto vertex_requester = robots.at(uwb->requester_id).new_vertex(sensor_type.range, uwb->header, optimizer);

    auto vertex_responder = robots.at(uwb->responder_id).new_vertex(sensor_type.range, uwb->header, optimizer);

    optimizer.addVertex(vertex_requester);

    optimizer.addVertex(vertex_responder);

    auto edge = new g2o::EdgeSE3Range();

    edge->vertices()[0] = vertex_requester;

    edge->vertices()[1] = vertex_responder;

    edge->setMeasurement(uwb->distance);

    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(1, 1);

    covariance(0,0) = pow(uwb->distance_err, 2);

    edge->setInformation(covariance.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    optimizer.addEdge(edge);

    // add imu edge
    if(uwb->requester_id == self_id)
    {
        auto last_vertex = robots.at(self_id).last_vertex(sensor_type.range);
        uwb_number = uwb_number + 1;
    
        sensor_msgs::Imu Imu(*Imu_);
        // calculate translation
        double qx = Imu.orientation.x;
        double qy = Imu.orientation.y;
        double qz = Imu.orientation.z;
        double qw = Imu.orientation.w;

        Matrix3d T_matrix;
        T_matrix <<1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw,
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
            last_vertex_velocity = last_vertex->estimate().translation() - last_last_vertex->estimate().translation();
            last_vertex_velocity = last_vertex_velocity/dt;
        }

        last_last_vertex = last_vertex;

        Vector3d translation = last_vertex_velocity*dt+0.5*acceleration*pow(dt,2);

        Eigen::Isometry3d current_pose;
        Quaterniond imu_rotation = Quaterniond(Imu.orientation.w,Imu.orientation.x,Imu.orientation.y,Imu.orientation.z);
        current_pose.rotate(imu_rotation);
        current_pose.translate(g2o::Vector3D(0, 0, 0) );

        // calculate transform matrix
        Isometry3d transform_matrix = last_vertex->estimate().inverse()*current_pose;
        translation = current_pose.rotation().inverse()*translation;
        transform_matrix(0,3) = translation[0];
        transform_matrix(1,3) = translation[1];
        transform_matrix(2,3) = translation[2];

        g2o::EdgeSE3 *SE3edge = new g2o::EdgeSE3();
    
        SE3edge->vertices()[0] = last_vertex;

        SE3edge->vertices()[1] = vertex_requester; 

        SE3edge->setMeasurement(transform_matrix);

        Eigen::Map<Eigen::MatrixXd> covariance_orientation(Imu.orientation_covariance.data(), 3, 3);
        Eigen::Map<Eigen::MatrixXd> covariance_translation(Imu.linear_acceleration_covariance.data(), 3, 3);
        Eigen::Matrix3d zero_matrix = Eigen::Matrix3d::Zero();

        Eigen::MatrixXd SE3information(6,6);

        SE3information << covariance_orientation,zero_matrix,
                      zero_matrix,covariance_translation;

        SE3edge->setInformation(SE3information.inverse());

        edge->setRobustKernel(new g2o::RobustKernelHuber());
    
        optimizer.addEdge( SE3edge );

    }

    ROS_INFO("added range edge id: %d", uwb->header.seq);

    solve();
}


void Localization::addImuEdge(const sensor_msgs::Imu::ConstPtr& Imu_)

{     
    
}


void Localization::addTwistEdge(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twist_)
{
    geometry_msgs::TwistWithCovarianceStamped twist(*twist_);

    double dt = twist.header.stamp.toSec() - robots.at(self_id).last_header().stamp.toSec();

    auto last_vertex = robots.at(self_id).last_vertex();

    auto new_vertex = robots.at(self_id).new_vertex(sensor_type.twist, twist.header, optimizer);

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->vertices()[0] = last_vertex;

    edge->vertices()[1] = new_vertex;

    Eigen::ArrayXXd covariance;

    auto measurement = twist2transform(twist.twist, covariance, dt);

    edge->setMeasurement(measurement);

    edge->setInformation(covariance.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    ROS_INFO("added twist edges with id: %d!", twist.header.seq);
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
