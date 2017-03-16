#include "uwb.h"

UWB::UWB(ros::NodeHandle n)
{

    sim_range_pub = n.advertise<uwb_driver::UwbRange>("/uwb_endorange_info", 1);

    // For g2o optimizer
    // do this once after you created the optimizer
    // add the camera parameters, caches are automatically resolved in the addEdge calls
    g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    optimizer.addParameter(cameraOffset);

    solver = new Solver();

    solver->setBlockOrdering(false);

    se3blockSolver = new SE3BlockSolver(solver);

    optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);

    optimizer.setAlgorithm(optimizationsolver);

    optimizer.setVerbose(true);

    accumulator = 0; 

    if(n.param("optimizer/maximum_iteration", iteration_max, 20))
        ROS_WARN("Using optimizer maximum iteration: %d", iteration_max);
        
    //uwb parameters reading
    if(n.getParam("/uwb/nodesId", nodesId))
        for (auto it:nodesId)
            ROS_WARN("Get node ID: %d", it);
    else
        ROS_ERROR("Can't get parameter nodesId from UWB");

    if(n.getParam("/uwb/nodesPos", nodesPos))
        for(auto it:nodesPos)
            ROS_WARN("Get node position: %4.2f", it);
    else
        ROS_ERROR("Can't get parameter nodesPos from UWB");

    if(n.getParam("/uwb/nodesRealPos", nodesRealPos))
        for(auto it:nodesRealPos)
            ROS_WARN("Get node real position: %4.2f", it);
    else
        ROS_ERROR("Can't get parameter nodesRealPos from UWB");
    
    if(n.getParam("/uwb/fixed", fixed))
        for(auto it:fixed)
            ROS_WARN("Fixed is: %s", it ? "true" : "false");
    else
        ROS_ERROR("Can't get parameter Fixed from UWB");

    if(n.getParam("/uwb/UWB_count", UWB_count))
        ROS_WARN("UWB_count is: %d", UWB_count);
    else
        ROS_ERROR("Can't get parameter UWB_count from UWB");

    sim_edge_map(UWB_count, nodesRealPos);//To generate the map structure to store the Measured ranges

}





void UWB::publish()
{
    uwb_driver::UwbRange uwbmsg; 
    ros::Rate loop_rate(100);

    for(int i = 0; i < 12 ; i++)
    {
        for(int j =0; j< 12; j++)
        {   
            uwbmsg.requester_idx = i%4;
            uwbmsg.responder_idx = j%4;
            uwbmsg.distance = add_noise(get_map_value(i%4,j%4),0.2,0.2);
            cout<<i%4 <<" and " << j%4 <<" range: "<< uwbmsg.distance << endl;
            sim_range_pub.publish(uwbmsg);
            ros::spinOnce();
        }
    }
}


void UWB::addRangeEdge(const uwb_driver::UwbRange::ConstPtr& uwbmsg)
{
   auto edge = create_range_edge(vertices[uwbmsg->responder_idx], vertices[uwbmsg->requester_idx], anchorMap[std::make_pair(uwbmsg->responder_idx,uwbmsg->requester_idx)], 1);
   
   optimizer.addEdge(edge);
   
   ROS_INFO("added range edge between responder: <%d> and requester: <%d> is distance: %lf", uwbmsg->responder_idx, uwbmsg->requester_idx, uwbmsg->distance);
}



void UWB::solve()
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

    ROS_INFO("Graph optimized with error: %f", optimizer.chi2());

    timer.toc();
}

void UWB::save()
{
    optimizer.save("/home/hao/catkin_ws/src/localization/data/simulatedanchor.g2o");
}

void UWB::addPriorEdge(int UWB_count)
{
    uwb_id = 0;
    
    while( uwb_id < UWB_count)
    {//setEstimate according to the idx and yaml file nodesPos 0 to 11 are used to set estimate SE3 vertex
        auto vertex_position = new g2o::VertexSE3(); 
        vertex_position->setEstimate(Eigen::Transform<double,3,1>(Eigen::Translation<double, 3>(nodesPos[0+3*uwb_id],nodesPos[1+3*uwb_id],nodesPos[2+3*uwb_id])));
        
        subfixed[0]=fixed[3*uwb_id];
        subfixed[1]=fixed[1+3*uwb_id];
        subfixed[2]=fixed[2+3*uwb_id];            
        
        vertex_position->setId(uwb_id); 

        vertices.push_back(vertex_position); 
        
        optimizer.addVertex(vertex_position);

        auto edge = create_prior_edge(vertex_position);
        
        optimizer.addEdge(edge);
        
        ROS_INFO("added 1 requester prior edge with id: <%d>;", uwb_id);

        uwb_id +=1;
    }
}

inline g2o::EdgeSE3Prior* UWB::create_prior_edge(g2o::VertexSE3* vertex)
{
    auto edge = new g2o::EdgeSE3Prior();

    Matrix6d pinfo = Matrix6d::Zero();

    for(int i=0;i<3;i++)
    {
        if(subfixed[i]) //if the bool subfixed[i] is true, meaning this axis is 100% confident, fix 
        {
            pinfo(i,i)=1e9;
        }
        else
        {
            pinfo(i,i)=0;
        }
    }

    edge->setInformation(pinfo);

    edge->setMeasurement(vertex->estimate());
    
    edge->vertices()[0] = vertex;

    edge->setParameterId(0,0); 

    return edge;
}




double UWB::add_noise(double value, double mean, double std_d)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    
    std::default_random_engine generator (seed);

    std::normal_distribution<double> distribution (mean,std_d);

    return (value+distribution(generator));
}


void UWB::sim_edge_map(int UWB_count, std::vector<double> nodesRealPos)
{
    double rangeCalculated[UWB_count][UWB_count]={{0}}; 
   
    for(int i = 0; i < UWB_count; ++i) //from the ith anchor
    {
        for(int j = 0; j < UWB_count; ++j) //to the jth anchor
        {
            rangeCalculated[i][j] = sqrt(pow((nodesRealPos[i*3]-nodesRealPos[j*3]),2)+pow((nodesRealPos[1+i*3]-nodesRealPos[1+j*3]),2)+pow((nodesRealPos[2+i*3]-nodesRealPos[2+j*3]),2));
        
            anchorMap[std::make_pair(i,j)] = rangeCalculated[i][j];
        }
    }
}


inline g2o::EdgeSE3Range* UWB::create_range_edge(g2o::VertexSE3* vertex1, g2o::VertexSE3* vertex2, double distance, double covariance)
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


double UWB::get_map_value(int i, int j)
{
    return anchorMap[std::make_pair(i,j)];
}