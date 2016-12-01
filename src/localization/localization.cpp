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

Localization::Localization() 
{
    solver = new Solver();

    solver->setBlockOrdering(false);

    se3blockSolver = new SE3BlockSolver(solver);

    optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);

    optimizer.setAlgorithm(optimizationsolver);

    optimizer.setVerbose(true);

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

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    auto last_vertex = robots.at(self_id).last_vertex(sensor_type.slam);

    auto new_vertex  = robots.at(self_id).new_vertex(sensor_type.slam, pose_cov_->header, optimizer);

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


void Localization::addRangeEdge(const uwb_driver::UwbRange::ConstPtr& uwb)
{
    robots.emplace(uwb->requester_id, Robot(uwb->requester_idx, true, optimizer));

    robots.emplace(uwb->responder_id, Robot(uwb->responder_idx, true, optimizer));

    auto vertex_requester = robots.at(uwb->requester_id).new_vertex(sensor_type.uwb, uwb->header, optimizer);

    auto vertex_responder = robots.at(uwb->responder_id).new_vertex(sensor_type.uwb, uwb->header, optimizer);

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

    ROS_INFO("added range edge id: %d", uwb->header.seq);

}

void Localization::addImuEdge(const sensor_msgs::Imu::ConstPtr& imu)
{

}
