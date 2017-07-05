#include <iostream>
#include <iomanip>
#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/RayDirnServer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/LaserUtils.h>

using namespace lidar_sim;

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // todo: delete me
    // std::vector<double> vec{1, 2, 3};
    // double theta, phi, r;
    // std::tie(theta, phi, r) = cart2sph(vec);
    // std::cout << "theta: " << theta << " phi: " << phi << " r: " << r << std::endl; exit(0);

    // load section
    int section_sim_id = 3;
    std::string rel_path_section = genRelPathSection(section_sim_id);
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // laser calib
    LaserCalibParams laser_calib_params;

    RayDirnServer ray_dirn_server;

    size_t packet_idx = 0;

    // pose, ray origin
    double t = section.m_packet_timestamps[packet_idx];
    std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, laser_calib_params);
    // packet pts
    std::vector<std::vector<double> > this_pts = section.getPtsAtTime(t);


    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
