#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load ellipsoids
    std::string rel_path_ellipsoid_models = "data/ellipsoid_models.txt";
    EllipsoidModels ellipsoid_models = 
	loadEllipsoidModelsFromFile(rel_path_ellipsoid_models);

    // load section
    std::string rel_path_section = "data/section_08_world_frame_subsampled_timed.xyz";
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    int n_poses_to_sim = 500;

    std::vector<std::vector<double> > imu_poses;
    double t_max = section.m_pt_timestamps.back();
    double t_min = section.m_pt_timestamps[0];
    double dt = (t_max-t_min)/n_poses_to_sim;

    // get poses
    double t = t_min;
    while(t < t_max)
    {
	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
	imu_poses.push_back(imu_pose);
	t += dt;
    }
    
    // sim object
    const LaserCalibParams laser_calib_params;
    EllipsoidModelSim sim;
    sim.setEllipsoidModels(ellipsoid_models);
    sim.setLaserCalibParams(laser_calib_params);

    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    std::tie(sim_pts_all, hit_flag) = sim.simPtsGivenPoses(imu_poses);
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);

    // write sim pts
    std::string rel_path_output = "data/rim_stretch_veg_validation_sim.xyz";
    writePtsToXYZFile(sim_pts, rel_path_output);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}
