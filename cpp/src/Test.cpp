#include <iostream>

#include <lidar_sim/Test.h>
#include <lidar_sim/Visualizer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/EllipsoidsModelUtils.h>

using namespace lidar_sim;

bool Test::testVizPCD(std::string file_name, int is_rgb = 1)
{
    std::cout << "Visualizing: " << file_name << std::endl;
    Visualizer viz;
    viz.visualize(file_name, is_rgb);
    return true;
}

bool Test::testPoseServer(std::string rel_path_pose_log)
{
    PoseServer pose_server(rel_path_pose_log);
    for(size_t i = 0; i < 10; i++)
    	pose_server.printLogAtIndex(i);    
    return true;
}

bool Test::testGetPoseAtTime(std::string rel_path_pose_log, double t)
{
    PoseServer pose_server(rel_path_pose_log);
    std::vector<double> pose = pose_server.getPoseAtTime(t);
    pose_server.printPose(pose);
    return true;
}

bool Test::testCovMat()
{
    Pts pts { {1,2,3},
	{6,7,6},
	{5,1,5},
	{9,7,8}};
    
    std::vector<double> mu = calcPtsMean(pts);
    std::cout << "mu :" << std::endl;
    for(size_t i = 0; i < mu.size(); ++i)
	std::cout << mu[i] << " ";
    std::cout << std::endl;

    Pts centered_pts = calcCenteredPts(pts);
    std::cout << "centered :" << std::endl;
    for(size_t j = 0; j < centered_pts.size(); ++j)
    {
	for(size_t i = 0; i < 3; ++i)
	    std::cout << centered_pts[j][i] << " ";
	std::cout << std::endl;
    }

    Eigen::MatrixXd cov_mat = calcPtsCovMat(pts);
    std::cout << "cov mat: " << cov_mat << std::endl;

    return true;
}
