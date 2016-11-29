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


Localization::Localization( int N )
{
    solver = new Solver();

    solver->setBlockOrdering(false);

    se3blockSolver = new SE3BlockSolver(solver);

    optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);

    optimizer.setAlgorithm(optimizationsolver);

    optimizer.setVerbose(true);

    // for calculating initial position

    begin_optimizer.setAlgorithm(optimizationsolver);

    begin_optimizer.setVerbose(true);

    iteration_max = 100;

    number_of_nodes = N;

    // *last uwb information header.stamp
    laststamp = ros::Time(0.001);

    // variables used in member function Localization::setup;
    M1=1;
    M2=1; 
    linkmatrix = MatrixXd::Zero(5, 5); // linkmatrix
}


void Localization::solve()
{
    optimizer.initializeOptimization();

    optimizer.optimize(iteration_max);
}

void Localization::setup_initialization()
{
    size_t sum = number_of_nodes;

    for (size_t i = 0; i <= sum; i++)
    {        
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId(i);

        switch(i)
        {
            // initial position guess 
            case 0:
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,-0.5)));
            v->setFixed(true);
            break;

            case 1:
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(1.7,0,-1)));
            break;

            case 2:
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(2,1.1,-2.5)));    
            break;

            case 3:
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(1.3,1.4,-0.6)));
            break;

            case 4:
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(1.6,1.7,-2.5)));              
            break;
            
            // be used for fixed y and z calculating
            case 5:
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,0)));     
            v->setFixed(true);
            default:           
            break;

        }

        begin_optimizer.addVertex(v);
    }
   

    // add known z-value edge of anchor
    for (size_t j=1; j <sum;j++)
    {         
        g2o::zedge *edge = new g2o::zedge();

        edge->vertices()[0] = begin_optimizer.vertex(j);
        edge->vertices()[1] = begin_optimizer.vertex(sum);

        g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *> (begin_optimizer.vertex(j));
        double delta = v1->estimate().translation()[2];

        edge->setMeasurement(delta);
        
        Eigen::MatrixXd information = Eigen::MatrixXd::Zero(1, 1);
        information(0,0) = 0.1;
        edge->setInformation(information.inverse());

        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        begin_optimizer.addEdge( edge );          
        }

    // add known y-value edge of anchor   
    g2o::yedge *edge = new g2o::yedge();

    edge->vertices()[0] = begin_optimizer.vertex(1);
    edge->vertices()[1] = begin_optimizer.vertex(sum);

    g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *> (begin_optimizer.vertex(1));
    double delta = v1->estimate().translation()[1];

    edge->setMeasurement(delta);
        
    Eigen::MatrixXd information = Eigen::MatrixXd::Zero(1, 1);
    information(0,0) = 0.1;
    edge->setInformation(information.inverse());
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
    begin_optimizer.addEdge( edge ); 

}


void Localization::addPoseEdge(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_cov_)
{
    geometry_msgs::PoseWithCovarianceStamped pose_cov(*pose_cov_);  

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->vertices()[0] = optimizer.vertex(0);

    edge->vertices()[1] = optimizer.vertex(1);

    Eigen::Isometry3d measurement;

    tf::poseMsgToEigen(pose_cov.pose.pose, measurement);
    // change to Isometry3d

    edge->setMeasurement(measurement);

    Eigen::Map<Eigen::MatrixXd> covariance(pose_cov.pose.covariance.data(), 6, 6);

    edge->setInformation(covariance.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    optimizer.addEdge(edge);

    edges_pose.push_back(edge);

    ROS_INFO("added pose edge id: %d", pose_cov.header.seq);
}


// add IMU pose edge : fang xu

void Localization::IMU_UWB(const uwb_driver::UwbRange::ConstPtr& uwb_data_, const sensor_msgs::Imu::ConstPtr& Imu_data_)
{       
    sensor_msgs::Imu Imu_data(*Imu_data_);
    uwb_driver::UwbRange uwb_data(*uwb_data_);

    // *calculate time interval between two UWB measurement
    ros::Time current_stamp = uwb_data.header.stamp;
    ros::Time last_stamp = laststamp;
    double delta_T;

    //current seq for the uwb measurement
    int sequence = uwb_data.header.seq; 
    if (sequence<2)
    {        
        delta_T = 3/50;
        laststamp = current_stamp;
    }    
    else
    {   
        delta_T = (current_stamp- last_stamp).toSec();
        laststamp = current_stamp;
    }
    
    //add two vertices for the new uwb measurment
    int vertex0_id = number_of_nodes*sequence+uwb_data.requester_idx;
    int vertex1_id = number_of_nodes*sequence+uwb_data.responder_idx;

    g2o::VertexSE3* v0 = new g2o::VertexSE3();
    v0->setId(vertex0_id);

    g2o::VertexSE3* v1 = new g2o::VertexSE3();
    v1->setId(vertex1_id);

    // previous vertex estimate
    g2o::VertexSE3* v0_p = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(vertex0_id-number_of_nodes));

    v0->setEstimate(v0_p->estimate());

    optimizer.addVertex(v0);

    poses.push_back(v0);


    g2o::VertexSE3* v1_p = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(vertex1_id-number_of_nodes));
    
    v1->setEstimate(v1_p->estimate());

    optimizer.addVertex(v1);

    poses.push_back(v1);

    // add EdgeSE3Range edge
    g2o::EdgeSE3Range *edge = new g2o::EdgeSE3Range();

    edge->vertices()[0] = optimizer.vertex(vertex0_id);

    edge->vertices()[1] = optimizer.vertex(vertex1_id);

    edge->setMeasurement(uwb_data.distance);

    Eigen::MatrixXd information = Eigen::MatrixXd::Zero(1, 1);

    information(0,0) = 0.0025;

    edge->setInformation(information.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    optimizer.addEdge(edge);

    edges_range.push_back(edge);

    // calculate translation
    double qx = Imu_data.orientation.x;
    double qy = Imu_data.orientation.y;
    double qz = Imu_data.orientation.z;
    double qw = Imu_data.orientation.w;

    Matrix3d transition;
    transition(0,0) = 1-2*qy*qy-2*qz*qz; transition(0,1) = 2*qx*qy-2*qz*qw; transition(0,2) = 2*qx*qz+2*qy*qw;
    transition(1,0) = 2*qx*qy + 2*qz*qw; transition(1,1) = 1-2*qx*qx-2*qz*qz; transition(1,2) = 2*qy*qz-2*qx*qw;
    transition(2,0) = 2*qx*qz-2*qy*qw; transition(2,1) = 2*qy*qz + 2*qx*qw; transition(2,2) = 1-2*qx*qx-2*qy*qy;

    Vector3d imu_acceleration = Vector3d(Imu_data.linear_acceleration.x,Imu_data.linear_acceleration.y,Imu_data.linear_acceleration.z);
    Vector3d gravity = Vector3d(0,0,9.8);
    Vector3d acceleration = transition.inverse()*imu_acceleration+gravity;

    // *v(k-1)= (p(k-1)-p(k-2))/2;
    g2o::VertexSE3* v0_pp = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(vertex0_id-2*number_of_nodes));
    Vector3d pre_velocity = (v0_p->estimate().translation() - v0_pp->estimate().translation())/delta_T;

    Vector3d delta_distance = pre_velocity*delta_T+0.5*acceleration*pow(delta_T,2);

    Eigen::Isometry3d current_pose;
    Quaterniond imu_direction = Quaterniond(Imu_data.orientation.w,Imu_data.orientation.x,Imu_data.orientation.y,Imu_data.orientation.z);
    current_pose.rotate(imu_direction);
    current_pose.translate(g2o::Vector3D(0, 0, 0) );
    // last rotation   v0_p->estimate().rotation()
    // calculate transform matrix
    Isometry3d transform_matrix = v0_p->estimate().inverse()*current_pose;
    delta_distance = current_pose.rotation().inverse()*delta_distance;
    transform_matrix(0,3) = delta_distance[0];
    transform_matrix(1,3) = delta_distance[1];
    transform_matrix(2,3) = delta_distance[2];

     // add  edgeSE3 edge    
     g2o::EdgeSE3 *SE3edge = new g2o::EdgeSE3();
    
     SE3edge->vertices()[0] = optimizer.vertex(vertex0_id);

     SE3edge->vertices()[1] = optimizer.vertex(vertex0_id-number_of_nodes); 

     SE3edge->setMeasurement(transform_matrix);

     Eigen::MatrixXd SE3information = Eigen::MatrixXd::Identity(6, 6) * 0.01;

     information(3,3) = 0.0001;
     information(4,4) = 0.0001;
     information(5,5) = 0.0001;

     SE3edge->setInformation(SE3information.inverse());
    
     optimizer.addEdge( SE3edge );

     edges_pose.push_back(SE3edge);    
}

//subscribe range msg from uwb_as node

void Localization::addRangeEdge(const uwb_driver::UwbRange::ConstPtr& uwb)

{

    //current index for the measurement
    int measCount = uwb->header.seq;

    //add two vertices for the new measurment
    int vertex0_id = number_of_nodes*measCount+uwb->requester_idx;
    int vertex1_id = number_of_nodes*measCount+uwb->responder_idx;

    g2o::VertexSE3* v0 = new g2o::VertexSE3();
    v0->setId(vertex0_id);

    //previous vertex estimate  

    // but where to add the initial vertex pose,if there is no last vertex information

    g2o::VertexSE3* v0_p = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(vertex0_id-number_of_nodes));

    v0->setEstimate(v0_p->estimate());

    optimizer.addVertex(v0);

    poses.push_back(v0);

    g2o::VertexSE3* v1 = new g2o::VertexSE3();

    v1->setId(vertex1_id);

    v1->setEstimate((dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(vertex1_id-number_of_nodes)))->estimate());

    optimizer.addVertex(v1);

    poses.push_back(v1);

    //add edge

    g2o::EdgeSE3Range *edge = new g2o::EdgeSE3Range();

    edge->vertices()[0] = optimizer.vertex(vertex0_id);

    edge->vertices()[1] = optimizer.vertex(vertex1_id);

    edge->setMeasurement(uwb->distance);

    Eigen::MatrixXd information = Eigen::MatrixXd::Zero(1, 1);

    information(0,0) = 0.0025;

    edge->setInformation(information.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    optimizer.addEdge(edge);

    edges_range.push_back(edge);

}


// calculating initial position , for test : testing data from data.txt
// terminal command rosrun data path : to publish the data
void Localization::setup(const geometry_msgs::PoseStamped::ConstPtr& uwb_)
{   
    geometry_msgs::PoseStamped uwb(*uwb_);

    int vertex0_idx = uwb.pose.position.x;
    int vertex1_idx = uwb.pose.position.y;

    // length linkmatrix
    linkmatrix(vertex0_idx,vertex1_idx)= uwb.pose.position.z;

    // add EdgeSE3Range edge
    g2o::EdgeSE3Range *edge = new g2o::EdgeSE3Range();
    
    edge->vertices()[0] = begin_optimizer.vertex(vertex0_idx);
    edge->vertices()[1] = begin_optimizer.vertex(vertex1_idx);

    edge->setMeasurement(linkmatrix(vertex0_idx,vertex1_idx));       
    Eigen::MatrixXd information = Eigen::MatrixXd::Zero(1, 1);
    information(0,0) = 0.1;
    edge->setInformation(information.inverse());

    edge->setRobustKernel( new g2o::RobustKernelHuber() );
    begin_optimizer.addEdge( edge );

    begin_optimizer.initializeOptimization();
    begin_optimizer.optimize(iteration_max);

    MatrixXd optimizeResult(6,3);
    for(size_t i=0;i<5;i++)
    {
        g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *> (begin_optimizer.vertex(i));
        optimizeResult(i,0) = abs(v1->estimate().translation()[0]);
        optimizeResult(i,1) = abs(v1->estimate().translation()[1]);
        optimizeResult(i,2) = v1->estimate().translation()[2];
    }
    cout<< optimizeResult.matrix()<<'\n'<<endl;
    begin_optimizer.save( "begin_result.g2o" );
    // save optimizeResult matrix used as the true initial value
    std::ofstream output("initial position matrix.txt"); 
    for (int k=0; k<number_of_nodes; k++)
    {
        for (int l=0; l<3; l++)
        {
            output << optimizeResult(k,l)<< ",";
        }

        output << '\n' << endl;
    }
}

void Localization::setupsolve()
{   
    begin_optimizer.initializeOptimization();
    begin_optimizer.optimize(iteration_max);
}
