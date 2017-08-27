#include <tuple>
#include <ctime>

#include <vtkProperty.h>

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
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionModelSim.h>

using namespace lidar_sim;

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_subsampled.xyz";

    return ss.str();
}

std::string genRelPathSliceRealPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/slice_real_pts.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // laser calib params
    LaserCalibParams laser_calib_params;

    // load section
    int section_id = 3;
    std::string rel_path_section = genRelPathSection(section_id);
    SectionLoader section(rel_path_section);

    size_t packet_id_sim_start, packet_id_sim_end, packet_step;	
    packet_id_sim_start = 1;
    packet_id_sim_end = 100;
    packet_step = 10;
    std::vector<std::vector<double> > real_pts;
    for (size_t i = packet_id_sim_start; i < packet_id_sim_end; i += packet_step)
    {
	double t = section.m_packet_timestamps[i];
    	// pose, ray origin
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, laser_calib_params);

	// packet pts
    	std::vector<std::vector<double> > this_real_pts = section.getPtsAtTime(t);

    	// add to big list of real pts
    	real_pts.insert(real_pts.end(), this_real_pts.begin(), this_real_pts.end());
    }

    // write out
    std::string rel_path_pts = genRelPathSliceRealPts(section_id);
    writePtsToXYZFile(real_pts, rel_path_pts);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
