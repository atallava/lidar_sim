#include <iostream>

#include <lidar_sim/Test.h>
#include <lidar_sim/Visualizer.h>
#include <lidar_sim/PoseServer.h>

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
