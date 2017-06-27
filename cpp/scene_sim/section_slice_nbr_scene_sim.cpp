#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/NearestNeighborSim.h>

using namespace lidar_sim;

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    ss << "data/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

std::string genRelPathSectionPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

std::string genRelPathSectionPtsGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/ground_segmentation/section_pts_" << std::setw(2) << std::setfill('0') << section_id
       << "_ground.asc";

    return ss.str();
}

std::string genRelPathSimSceneObjectPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/nbr_sim/sim_scene_object_pts.xyz";
	
    return ss.str();
}

std::string genRelPathSliceRealPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/nbr_sim/slice_real_pts.xyz";

    return ss.str();
}

std::string genRelPathSimPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/nbr_sim/slice_sim_pts.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_sim_id = 4;
    std::string rel_path_section = genRelPathSection(section_sim_id);
    SectionLoader section(rel_path_section);

    // get sim scene pts
    std::string rel_path_scene_ground_pts = genRelPathSectionPtsGround(section_sim_id);
    std::vector<std::vector<double> > scene_ground_pts  = loadPtsFromXYZFile(rel_path_scene_ground_pts);
    std::string rel_path_sim_scene_object_pts = genRelPathSimSceneObjectPts(section_sim_id);
    std::vector<std::vector<double> > sim_scene_object_pts  = loadPtsFromXYZFile(rel_path_sim_scene_object_pts);
    std::vector<std::vector<double> > sim_scene_pts = scene_ground_pts;
    sim_scene_pts.insert(sim_scene_pts.end(),
			 sim_scene_object_pts.begin(), sim_scene_object_pts.end());
    
    // sim object
    NearestNeighborSim sim;
    sim.setTrainPts(sim_scene_pts);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // slice ids
    size_t packet_id_sim_start, packet_id_sim_end;
    packet_id_sim_start = 50000;
    packet_id_sim_end = packet_id_sim_start + 20000;

    // sim
    // loop over packets
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    std::vector<std::vector<double> > real_pts;
    std::vector<int> ellipsoid_blocks_queried;
    std::vector<int> triangle_blocks_queried;
    size_t packet_array_step = 10;
    for(size_t i = packet_id_sim_start; 
	i < packet_id_sim_end; i += packet_array_step)
    {
    	double t = section.m_packet_timestamps[i];

    	// pose, ray origin
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);

    	// packet pts
    	std::vector<std::vector<double> > this_pts = section.getPtsAtTime(t);

    	// add to big list of real pts
    	real_pts.insert(real_pts.end(), this_pts.begin(), this_pts.end());
	
    	// ray dirns
	// here is where you could alternately get directions from laser intrinsics
    	std::vector<std::vector<double> > ray_dirns  = calcRayDirns(ray_origin, this_pts);

	// todo: uncomment
    	// // simulate 
    	// std::vector<std::vector<double> > this_sim_pts;
    	// std::vector<int> this_hit_flag;
    	// std::tie(this_sim_pts, this_hit_flag) = sim.simPtsGivenRays(ray_origin, ray_dirns); 

    	// // add to big list of sim pts
    	// sim_pts_all.insert(sim_pts_all.end(), this_sim_pts.begin(), this_sim_pts.end());
    	// hit_flag.insert(hit_flag.end(), this_hit_flag.begin(), this_hit_flag.end());
    }

    // todo: uncomment
    // // weed out non-hits
    // std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);

    // write real pts
    std::string rel_path_real_pts = genRelPathSliceRealPts(section_sim_id);
    writePtsToXYZFile(real_pts, rel_path_real_pts);

    // todo: uncomment
    // // write sim pts
    // std::string rel_path_sim_pts = genRelPathSimPts(section_sim_id);
    // writePtsToXYZFile(sim_pts, rel_path_sim_pts);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
