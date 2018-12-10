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


Localization::Localization(ros::NodeHandle n)
{
    pose_realtime_pub = n.advertise<geometry_msgs::PoseStamped>("realtime/pose", 1);

    pose_optimized_pub = n.advertise<geometry_msgs::PoseStamped>("optimized/pose", 1);

    path_optimized_pub = n.advertise<nav_msgs::Path>("optimized/path", 1);

    number_measurements = 0;


// For g2o optimizer
    solver = new Solver();

    solver->setBlockOrdering(false);

    se3blockSolver = new SE3BlockSolver(solver);

    optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);

    optimizer.setAlgorithm(optimizationsolver);

    g2o::ParameterSE3Offset* zero_offset = new g2o::ParameterSE3Offset;
    zero_offset->setId(0);
    optimizer.addParameter(zero_offset);
    
    bool verbose_flag;
    if(n.param("optimizer/verbose", verbose_flag, false))
    {
        ROS_WARN("Using optimizer verbose flag: %s", verbose_flag ? "true":"false");
        optimizer.setVerbose(verbose_flag);
    }

    if(n.param("optimizer/maximum_iteration", iteration_max, 20))
        ROS_WARN("Using optimizer maximum iteration: %d", iteration_max);

    if(n.param("optimizer/minimum_optimize_error", minimum_optimize_error, 1000.0))
        ROS_WARN("Will skip estimation if optimization error is larger than: %f", minimum_optimize_error);

// For robots
    if(n.getParam("robot/trajectory_length", trajectory_length))
        ROS_WARN("Using robot trajectory_length: %d", trajectory_length);

    if(n.param("robot/maximum_velocity", robot_max_velocity, 1.0))
        ROS_WARN("Using robot maximum_velocity: %fm/s", robot_max_velocity);

    if(n.param("robot/distance_outlier", distance_outlier, 1.0))
        ROS_WARN("Using uwb outlier rejection distance: %fm", distance_outlier);


// For UWB initial position parameters reading
    if(!n.getParam("/uwb/nodesId", nodesId))
        ROS_ERROR("Can't get parameter nodesId from UWB");

    if(!n.getParam("/uwb/nodesPos", nodesPos))
        ROS_ERROR("Can't get parameter nodesPos from UWB");

    self_id = nodesId.back();
    ROS_WARN("Init self robot ID: %d with moving option", self_id);

    for (size_t i = 0; i < nodesId.size(); ++i)
    {
        if(n.hasParam("topic/relative_range")||self_id==nodesId[i])
        {
            robots.emplace(nodesId[i], Robot(nodesId[i], false, trajectory_length));
            ROS_WARN("robot ID %d is set moving", nodesId[i]);
        }
        else // for fixed anchor
            robots.emplace(nodesId[i], Robot(nodesId[i], true, 1));

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose(0,3) = nodesPos[i*3];
        pose(1,3) = nodesPos[i*3+1];
        pose(2,3) = nodesPos[i*3+2];
        robots.at(nodesId[i]).init(optimizer, pose);
        ROS_WARN("Init robot ID: %d with position (%.2f,%.2f,%.2f)", nodesId[i], pose(0,3), pose(1,3), pose(2,3));
    }

// for multi-antena with offsets
    std::vector<double> antennaOffset;
    if(n.getParam("/uwb/antennaOffset", antennaOffset))
    {
        ROS_WARN("Using %d antennas", antennaOffset.size()/3);
        offsets = std::vector<Eigen::Isometry3d>(antennaOffset.size()/3, Eigen::Isometry3d::Identity());
        for (size_t i = 0; i < antennaOffset.size()/3; ++i)
        {
            offsets[i](0,3) = antennaOffset[i*3];
            offsets[i](1,3) = antennaOffset[i*3+1];
            offsets[i](2,3) = antennaOffset[i*3+2];
            ROS_WARN("Init antenna ID: %d with position (%.2f,%.2f,%.2f)", i+1,offsets[i](0,3), offsets[i](1,3), offsets[i](2,3));
        }
    }

// For Debug
    if(n.getParam("log/filename_prefix", name_prefix))
        if(antennaOffset.size() > 0)
            set_file(antennaOffset);
        else
            set_file();
    else
        ROS_WARN("Won't save any log files.");

    if(n.param<string>("frame/target", frame_target, "estimation"))
        ROS_WARN("Using topic target frame: %s", frame_target.c_str());

    if(n.param<string>("frame/source", frame_source, "local_origin"))
        ROS_WARN("Using topic source frame: %s", frame_source.c_str());

    if(n.param<bool>("publish_flag/tf", publish_tf, false))
        ROS_WARN("Using publish_flag/tf: %s", publish_tf ? "true":"false");

    if(n.param<bool>("publish_flag/range", publish_range, false))
        ROS_WARN("Using publish_flag/range: %s", publish_range ? "true":"false");

    if(n.param<bool>("publish_flag/pose", publish_pose, false))
        ROS_WARN("Using publish_flag/pose: %s", publish_pose ? "true":"false");

    if(n.param<bool>("publish_flag/twist", publish_twist, false))
        ROS_WARN("Using publish_flag/twist: %s", publish_twist ? "true":"false");
	
	if(n.param<bool>("publish_flag/lidar", publish_lidar, false))
        ROS_WARN("Using publish_flag/lidar: %s", publish_lidar ? "true":"false");

    if(n.param<bool>("publish_flag/imu", publish_imu, false))
        ROS_WARN("Using publish_flag/imu: %s", publish_imu ? "true":"false");

    if(n.param<bool>("publish_flag/relative_range", publish_relative_range, false))
        ROS_WARN("Using publish_flag/relative_range: %s", publish_relative_range ? "true":"false");

}


void Localization::solve()
{
    timer.tic();

    optimizer.initializeOptimization();

    optimizer.optimize(iteration_max);

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
    // ROS_INFO("Graph optimized with error: %f", optimizer.chi2());

    // g2o::SparseBlockMatrix<MatrixXd> spinv;

    // if(optimizer.computeMarginals(spinv, robots.at(self_id).last_vertex()))
    //     cout<<spinv.block(0,0)<<endl;    
    // else
    //     cout<<"can't compute"<<endl;

    timer.toc();
}


void Localization::publish()
{
    double error = optimizer.chi2();

    if (error < minimum_optimize_error)
        ROS_INFO("Graph optimized with error: %f ", error);
    else
    {
        ROS_WARN("Skip optimization with error: %f ", error);
        return;
    }
    

    auto pose = robots.at(self_id).current_pose();

    pose.header.frame_id = frame_source;

    pose_realtime_pub.publish(pose);

    auto path = robots.at(self_id).vertices2path();

    path->header.frame_id = frame_source;

    path_optimized_pub.publish(*path);

    pose_optimized_pub.publish(path->poses[trajectory_length/2]);

    if(flag_save_file)
    {
        save_file(pose, realtime_filename);
        save_file(path->poses[trajectory_length/2], optimized_filename);        
    }

    if(publish_tf)
    {
        if(publish_relative_range)
        {
            for (size_t i = 0; i < nodesId.size(); ++i)
            {
                auto pose = robots.at(nodesId[i]).current_pose();

                pose.header.frame_id = frame_source;

                tf::poseMsgToTF(pose.pose, transform);
                
                br.sendTransform(tf::StampedTransform(transform, pose.header.stamp, frame_source, "rl_" + std::to_string(nodesId[i]))); 
            }
        }
        else
        {
            tf::poseMsgToTF(pose.pose, transform);
            br.sendTransform(tf::StampedTransform(transform, pose.header.stamp, frame_source, frame_target));
        }

    }

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

    edge->setRobustKernel(new g2o::RobustKernelCauchy());

    optimizer.addEdge(edge);

    ROS_INFO("added pose edge id: %d frame_id: %s;", pose_cov.header.seq, pose_cov.header.frame_id.c_str());

    if (publish_pose)
    {
        solve();
        publish();
    }
}





#ifdef TIME_DOMAIN
void Localization::addRangeEdge(const uwb_driver::UwbRange::ConstPtr& uwb)
#else
void Localization::addRangeEdge(const bitcraze_lps_estimator::UwbRange::ConstPtr& uwb)
#endif
{

    ++number_measurements;


    double distance_estimation= (robots.at(uwb->requester_id).last_vertex()->estimate().translation() -
                                 robots.at(uwb->responder_id).last_vertex()->estimate().translation()).norm();

    if (number_measurements > trajectory_length && abs(distance_estimation-uwb->distance) > distance_outlier)
    {
        ROS_WARN("Reject ID: %d measurement: %fm", uwb->responder_id, uwb->distance);
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

    if( (frame_id.find(uwb->header.frame_id)!=string::npos) || (frame_id.find("none")!=string::npos))
    {    
        auto vertex_requester = robots.at(uwb->requester_id).new_vertex(sensor_type.range, uwb->header, optimizer);

        auto edge = create_range_edge(vertex_requester, vertex_responder, uwb->distance, distance_cov);

        if(uwb->antenna > 0)
            edge->setVertexOffset(0, offsets[uwb->antenna-1]);
        
        optimizer.addEdge(edge);

        auto edge_requester_range = create_range_edge(vertex_last_requester, vertex_requester, 0, cov_requester);

        optimizer.addEdge(edge_requester_range); 

        if(uwb->antenna > 0)
            ROS_INFO("added two requester range edge on id: <%d> with offsets %d <%.2f, %.2f, %.2f> at %.3f;",
                uwb->responder_id, uwb->antenna-1, offsets[uwb->antenna-1](0,3), offsets[uwb->antenna-1](1,3), offsets[uwb->antenna-1](2,3), dt_requester);
        else
            ROS_INFO("added two requester range edge on id: <%d> ", uwb->responder_id);
    }
    else
    {
        auto edge = create_range_edge(vertex_last_requester, vertex_responder, uwb->distance, distance_cov + cov_requester);

        optimizer.addEdge(edge); // decrease computation

        ROS_INFO("added requester edge with id: <%d>", uwb->responder_id);


    }


    if (!robots.at(uwb->responder_id).is_static())
    {
        double cov_responder = pow(robot_max_velocity*dt_responder/3, 2); //3 sigma priciple

        auto edge_responder_range = create_range_edge(vertex_last_responder, vertex_responder, 0, cov_responder);

        optimizer.addEdge(edge_responder_range);

        ROS_INFO("added responder trajectory edge;");
    }
        
    if (publish_range && number_measurements > trajectory_length)
    {
        solve();
        publish();
    }
}

#ifdef RELATIVE_LOCALIZATION
void Localization::addRLRangeEdge(const uwb_reloc::uwbTalkData::ConstPtr& uwb)
{
    std_msgs::Header RLheader;
    RLheader.stamp  = uwb->time_stamp;
    RLheader.frame_id = "uwb";

    double dt_requester = uwb->time_stamp.toSec() - robots.at(uwb->rqstrId).last_header().stamp.toSec();
    double dt_responder = uwb->time_stamp.toSec() - robots.at(uwb->rspdrId).last_header().stamp.toSec();

    double distance_cov = pow(0.054, 2);
    double cov_requester = pow(robot_max_velocity*dt_requester/3, 2); //3 sigma priciple
    double cov_responder = pow(robot_max_velocity*dt_responder/3, 2); //3 sigma priciple

    auto vertex_last_requester = robots.at(uwb->rqstrId).last_vertex();
    auto vertex_last_responder = robots.at(uwb->rspdrId).last_vertex();

    auto vertex_requester = robots.at(uwb->rqstrId).new_vertex(sensor_type.range, RLheader, optimizer);
    auto vertex_responder = robots.at(uwb->rspdrId).new_vertex(sensor_type.range, RLheader, optimizer);

    auto edge = create_range_edge(vertex_requester, vertex_responder, uwb->d, distance_cov);   
    optimizer.addEdge(edge);

    if (!robots.at(uwb->rspdrId).is_static())
    {
        auto edge_responder_range = create_range_edge(vertex_last_responder, vertex_responder, 0, cov_responder);
        optimizer.addEdge(edge_responder_range);
        ROS_INFO("added responder trajectory edge;");
    }

    // add EdgeSE3 using velocity information
    if (!robots.at(uwb->rqstrId).is_static())
    {
        g2o::EdgeSE3 *edge_requester = new g2o::EdgeSE3();
        edge_requester->vertices()[0] = vertex_last_requester;
        edge_requester->vertices()[1] = vertex_requester;

        Eigen::Isometry3d measurement;
        measurement.setIdentity();        
        measurement.translate(dt_requester*g2o::Vector3D(uwb->rqstr_vx, uwb->rqstr_vy, uwb->rqstr_vz));

        edge_requester->setMeasurement(measurement);

        Eigen::MatrixXd requester_SE3information = MatrixXd::Zero(6,6);
        requester_SE3information(0,0) = 1.0/cov_requester;
        requester_SE3information(1,1) = 1.0/cov_requester;
        requester_SE3information(2,2) = 1.0/cov_requester;

        edge_requester->setInformation(requester_SE3information);
        optimizer.addEdge(edge_requester);
    }

    if (publish_relative_range)
    {
        solve();
        publish();
    }
}
#endif

void Localization::addTwistEdge(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twist_)
{
    geometry_msgs::TwistWithCovarianceStamped twist(*twist_);

    double dt = twist.header.stamp.toSec() - robots.at(self_id).last_header().stamp.toSec();

    auto last_vertex = robots.at(self_id).last_vertex();

    auto new_vertex = robots.at(self_id).new_vertex(sensor_type.twist, twist.header, optimizer);

    auto edge = create_se3_edge_from_twist(last_vertex, new_vertex, twist.twist, dt);

    optimizer.addEdge(edge);

    ROS_INFO("added twist edge id: %d", twist.header.seq);

    if (publish_twist)
    {
        solve();
        publish();
    }
}


void Localization::addLidarEdge(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_cov_)
{
    if (robots.at(self_id).last_header().frame_id.find(pose_cov_->header.frame_id) == string::npos)
    {
        robots.at(self_id).append_last_header(pose_cov_->header.frame_id);

        auto last_vertex = robots.at(self_id).last_vertex(sensor_type.range);

        Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();

        current_pose = last_vertex->estimate();

        current_pose(2, 3) = pose_cov_->pose.pose.position.z;

        last_vertex->setEstimate(current_pose);

        Eigen::MatrixXd  information = Eigen::MatrixXd::Zero(6,6);
        information(2,2)= 1/0.05;
        
        g2o::EdgeSE3Prior* edgeprior = new g2o::EdgeSE3Prior();
        edgeprior->setInformation(information);
        edgeprior->vertices()[0]= last_vertex; 
        edgeprior->setMeasurement(current_pose); 
        edgeprior->setParameterId(0,0);
        optimizer.addEdge(edgeprior);

        ROS_INFO("added lidar edge id: %d", pose_cov_->header.seq);
    }

    if (publish_lidar)
    {
        solve();
        publish();
    }
}


void Localization::addImuEdge(const sensor_msgs::Imu::ConstPtr& Imu_)
{
    if (robots.at(self_id).last_header().frame_id.find(Imu_->header.frame_id) == string::npos)
    {
        robots.at(self_id).append_last_header(Imu_->header.frame_id);

        auto last_vertex = robots.at(self_id).last_vertex(sensor_type.range);

        Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();

        current_pose.rotate(Quaterniond(Imu_->orientation.w, Imu_->orientation.x, Imu_->orientation.y, Imu_->orientation.z));

        current_pose.translation() = last_vertex->estimate().translation();

        last_vertex->setEstimate(current_pose);

        Eigen::MatrixXd  information = Eigen::MatrixXd::Zero(6,6);
        information(3,3)= 1.0/Imu_->orientation_covariance[0];
        information(4,4)= 1.0/Imu_->orientation_covariance[4];
        information(5,5)= 1.0/Imu_->orientation_covariance[8];// roll, pitch, yaw
        
        g2o::EdgeSE3Prior* edgeprior = new g2o::EdgeSE3Prior();
        edgeprior->setInformation(information);
        edgeprior->vertices()[0]= last_vertex; 
        edgeprior->setMeasurement(current_pose); 
        edgeprior->setParameterId(0,0);
        optimizer.addEdge(edgeprior);

        ROS_INFO("added IMU edge id: %d", Imu_->header.seq);
    }

    if (publish_imu)
    {   
        solve();
        publish();
    }
}



void Localization::configCallback(localization::localizationConfig &config, uint32_t level)
{
    ROS_WARN("Get publish_optimized_poses: %s", config.publish_optimized_poses? "ture":"false");

    if (config.publish_optimized_poses)
    {
        ROS_WARN("Publishing Optimized poses");

        auto path = robots.at(self_id).vertices2path();
        
        for (int i = trajectory_length/2; i < trajectory_length; ++i)
        {
            pose_optimized_pub.publish(path->poses[i]);

            usleep(10000);
        }
        ROS_WARN("Published. Done");
    }
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

    edge->setRobustKernel(new g2o::RobustKernelCauchy());

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

    edge->setRobustKernel(new g2o::RobustKernelCauchy());

    return edge;
}


inline void Localization::save_file(geometry_msgs::PoseStamped pose, string filename)
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


void Localization::set_file()
{
    flag_save_file = true;
    char s[30];
    struct tm tim;
    time_t now;
    now = time(NULL);
    tim = *(localtime(&now));
    strftime(s,30,"_%Y_%b_%d_%H_%M_%S.txt",&tim);
    realtime_filename = name_prefix+"_realtime" + string(s);
    optimized_filename = name_prefix+"_optimized" + string(s);

    file.open(realtime_filename.c_str(), ios::trunc|ios::out);
    file<<"# "<<"iteration_max:"<<iteration_max<<"\n";
    file<<"# "<<"trajectory_length:"<<trajectory_length<<"\n";
    file<<"# "<<"maximum_velocity:"<<robot_max_velocity<<"\n";
    file.close();

    file.open(optimized_filename.c_str(), ios::trunc|ios::out);
    file<<"# "<<"iteration_max:"<<iteration_max<<"\n";
    file<<"# "<<"trajectory_length:"<<trajectory_length<<"\n";
    file<<"# "<<"maximum_velocity:"<<robot_max_velocity<<"\n";
    file.close();

    ROS_WARN("Loging to file: %s",realtime_filename.c_str());
    ROS_WARN("Loging to file: %s",optimized_filename.c_str());
}

void Localization::set_file(std::vector<double> antennaOffset)
{
    flag_save_file = true;
    char s[30];
    struct tm tim;
    time_t now;
    now = time(NULL);
    tim = *(localtime(&now));
    strftime(s,30,"_%Y_%b_%d_%H_%M_%S.txt",&tim);
    realtime_filename = name_prefix+"_realtime" + string(s);
    optimized_filename = name_prefix+"_optimized" + string(s);

    file.open(realtime_filename.c_str(), ios::trunc|ios::out);
    file<<"# "<<"iteration_max:"<<iteration_max<<"\n";
    file<<"# "<<"trajectory_length:"<<trajectory_length<<"\n";
    file<<"# "<<"maximum_velocity:"<<robot_max_velocity<<"\n";
    file.close();

    file.open(optimized_filename.c_str(), ios::trunc|ios::out);
    file<<"# "<<"iteration_max:"<<iteration_max<<"\n";
    file<<"# "<<"trajectory_length:"<<trajectory_length<<"\n";
    file<<"# "<<"maximum_velocity:"<<robot_max_velocity<<"\n";
    file.close();

    file.open(optimized_filename.c_str(), ios::trunc|ios::out);
    file<<"# "<<"antenna offsets: ";
    for(unsigned int i = 0; i < antennaOffset.size() - 1; i++)
        file << antennaOffset[i] << ",";
    file << antennaOffset[antennaOffset.size()-1] << "\n";
    file.close();

    ROS_WARN("Loging to file: %s",realtime_filename.c_str());
    ROS_WARN("Loging to file: %s",optimized_filename.c_str());
}

Localization::~Localization()
{
    if (flag_save_file)
    {
        auto path = robots.at(self_id).vertices2path();
        for (int i = trajectory_length/2; i < trajectory_length; ++i)
            save_file(path->poses[i], optimized_filename);
        cout<<"Results Loged to file: "<<optimized_filename<<endl;
    }
}
