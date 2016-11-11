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
}


void Localization::solve()
{
    optimizer.initializeOptimization();

    optimizer.optimize(iteration_max);
}


void Localization::addSlamEdge(geometry_msgs::PoseWithCovarianceStamped pose_cov)
{

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();

    edge->vertices()[0] = optimizer.vertex(0);

    edge->vertices()[1] = optimizer.vertex(1);

    Eigen::Isometry3d measurement;

    tf::poseMsgToEigen(pose_cov.pose.pose, measurement);

    edge->setMeasurement(measurement);

    Eigen::Map<Eigen::MatrixXd> covariance(pose_cov.pose.covariance.data(), 6, 6);

    edge->setInformation(covariance.inverse());

    edge->setRobustKernel(new g2o::RobustKernelHuber());

    optimizer.addEdge(edge);

    edges_slam.push_back(edge);

    // cout << "measurement" << *edge << endl;

}


float RAND(){ return 0.1*float(1.0*rand()/RAND_MAX);}

int test()
{
    Solver *solver = new Solver();
    solver->setBlockOrdering(false);
    SE3BlockSolver *se3blockSolver = new SE3BlockSolver(solver);
    g2o::OptimizationAlgorithmLevenberg *optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(optimizationsolver);
    optimizer.setVerbose(true);


    size_t num = 5;
    for (size_t i = 0; i < num; i++)
    {
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId(i);
        if (i==0)
        {
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(3,3,3)));
        }
        if (i==1)
        {
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,0)));
            v->setFixed(true);
        }
        if (i==2)
        {
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(1,0,0)));
            v->setFixed(true);
        }
        if (i==3)
        {
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,1,0)));
            v->setFixed(true);
        }
        if (i==4)
        {
            v->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,1)));
            v->setFixed(true);
        }
        optimizer.addVertex(v);
    }
    vector<g2o::EdgeSE3Range*> edges;
    for (size_t i = 1; i <=4 ; ++i)
    {
        g2o::EdgeSE3Range *edge = new g2o::EdgeSE3Range();

        float measurement;
        if(i==1)
        {
            edge->vertices()[0] = optimizer.vertex(0);
            edge->vertices()[1] = optimizer.vertex(i);
            measurement = sqrt(3)+RAND();
            edge->setMeasurement(measurement);
        }
        if (i>1&&i<=4)
        {
            edge->vertices()[0] = optimizer.vertex(0);
            edge->vertices()[1] = optimizer.vertex(i);
            measurement = sqrt(2)+RAND();
            edge->setMeasurement(measurement);
        }
        cout << "measurement id " << i << endl << measurement << endl;

        // if (i==5)
        // {
        //     edge->vertices()[0] = optimizer.vertex(1);
        //     edge->vertices()[1] = optimizer.vertex(2);
        //     edge->setMeasurement(1.414);
        // }

        // if (i==6)
        // {
        //     edge->vertices()[0] = optimizer.vertex(2);
        //     edge->vertices()[1] = optimizer.vertex(3);
        //     edge->setMeasurement(1.414);
        // }

        // if (i==7)
        // {
        //     edge->vertices()[0] = optimizer.vertex(3);
        //     edge->vertices()[1] = optimizer.vertex(4);
        //     edge->setMeasurement(1.414);
        // }

        // if (i==8)
        // {
        //     edge->vertices()[0] = optimizer.vertex(4);
        //     edge->vertices()[1] = optimizer.vertex(1);
        //     edge->setMeasurement(1.414);
        // }
        
        Eigen::MatrixXd information = Eigen::MatrixXd::Zero(1, 1);
        information(0,0) = 0.01;
        edge->setInformation(information.inverse());

        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

    // optimizer.save( "result_before.g2o" );
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    // optimizer.save( "result_after.g2o" );

    for (size_t i = 0; i < num; i++)
    {
        g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *> (optimizer.vertex(i));
        // g2o::SBACam pos = v->estimate();
        cout << "vertex id " << i << " Pose=" << endl << v->estimate().matrix()<< endl;
    }

    int inliers = 0;
    for ( auto e:edges )
    {
        e->computeError();
        cout<<"error = "<<e->chi2()<<endl;
        if ( e->chi2() > 1 )
        {
            cout<<"error = "<<e->chi2()<<endl;
        }
        else
        {
            inliers++;
        }
    }

    cout << "inliers in total points: " << inliers << "/" << edges.size() << endl;


    return 0;
}



// int main( int argc, char** argv )
// {
//     typedef g2o::BlockSolver_6_3 SlamBlockSolver;
//     typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

//     SlamLinearSolver *linearSolver = new SlamLinearSolver();
//     linearSolver->setBlockOrdering(false);
//     SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
//     g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

//     g2o::SparseOptimizer optimizer;
//     optimizer.setAlgorithm(solver);
//     optimizer.setVerbose( true );

//     size_t num = 100;
//     for (size_t i = 0; i < num; i++)
//     {
//         g2o::VertexSE3 *v = new g2o::VertexSE3();
//         v->setId(i);
//         v->setEstimate(Eigen::Isometry3d::Identity());
//         if (i==0)
//             v->setFixed(true);
//         optimizer.addVertex( v );
//     }



//     vector<g2o::EdgeSE3 *> edges;
//     for (size_t i = 0; i < num-1; ++i)
//     {
//         g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
//         // edge->vertices()[0] = optimizer.vertex(i);
//         // edge->vertices()[1] = optimizer.vertex(i + 1);
//        edge->setVertex(0, dynamic_cast<g2o::VertexSE3 *>   (optimizer.vertex(i)));
//        edge->setVertex(1, dynamic_cast<g2o::VertexSE3 *>   (optimizer.vertex(i + 1)));

//         Eigen::Isometry3d pose;
//         pose.setIdentity();
//         pose.translate(g2o::Vector3D(1, 0, 0) + 0.1 * g2o::Vector3D::Random());

//         cout << "measurement id " << i << " Pose=" << endl << pose.matrix() << endl;
//         edge->setMeasurement(pose);
//         Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) * 0.01;
//         information(3,3) = 0.0001;
//         information(4,4) = 0.0001;
//         information(5,5) = 0.0001;

//         edge->setInformation(information.inverse());
//         cout << "information id " << i << " information=" << endl << information.inverse() << endl;

// //        edge->setParameterId(0, parameterse3offset->id());
//         edge->setRobustKernel( new g2o::RobustKernelHuber() );
//         optimizer.addEdge( edge );
//         edges.push_back(edge);
//     }

//     g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
//     edge->vertices()[0] = optimizer.vertex(0);
//     edge->vertices()[1] = optimizer.vertex(99);
//     Eigen::Isometry3d pose;
//     pose.setIdentity();
//     pose.translate(g2o::Vector3D(99, 0, 0));// + 0.01 * g2o::Vector3D::Random());
//     edge->setMeasurement(pose);
//     Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) * 0.01;
//     information(3,3) = 0.0001;
//     information(4,4) = 0.0001;
//     information(5,5) = 0.0001;
//     edge->setInformation(information.inverse());
//     optimizer.addEdge( edge );
//     edges.push_back(edge);

//     optimizer.save( "result_before.g2o" );
//     optimizer.setVerbose(true);
//     optimizer.initializeOptimization();
//     optimizer.optimize(100);
//     optimizer.save( "result_after.g2o" );


//     for (size_t i = 0; i < num; i++)
//     {
//         g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *> (optimizer.vertex(i));
//         Eigen::Isometry3d pos = v->estimate();
//         cout << "vertex id " << i << " Pose=" << endl << pos.matrix() << endl;
//     }

//     cout <<"vertex done."<<endl;

//     int inliers = 0;
//     for ( auto e:edges )
//     {
//         e->computeError();
//         cout<<"error = "<<e->chi2()<<endl;
//         if ( e->chi2() > 1 )
//         {
//             cout<<"error = "<<e->chi2()<<endl;
//         }
//         else
//         {
//             inliers++;
//         }
//     }

//     cout << "inliers in total points: " << inliers << "/" << edges.size() << endl;
//     return 0;
// }