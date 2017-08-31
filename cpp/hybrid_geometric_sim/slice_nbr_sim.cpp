#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/NearestNeighborSim.h>
#include <lidar_sim/SimDetail.h>
#include <lidar_sim/RayDirnServer.h>

using namespace lidar_sim;

std::string genRelPathSectionPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

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

std::string genRelPathSimDetail(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/nbr_sim/slice_sim_detail.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_sim_id = 8;
    std::string rel_path_section = genRelPathSection(section_sim_id);
    SectionLoader section(rel_path_section);

    // sim object
    int section_id_train_pts = 3;
    NearestNeighborSim sim;
    std::string rel_path_sim_train_pts = genRelPathSectionPts(section_id_train_pts);
    sim.loadTrainPts(rel_path_sim_train_pts);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // slice ids
    size_t packet_id_sim_start, packet_id_sim_end;
    packet_id_sim_start = 1000;
    packet_id_sim_end = packet_id_sim_start + 1000;

    RayDirnServer ray_dirn_server;

    // sim
    // loop over packets
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    std::vector<std::vector<double> > real_pts;
    std::vector<int> ellipsoid_blocks_queried;
    std::vector<int> triangle_blocks_queried;
    SimDetail sim_detail;
    size_t packet_array_step = 500;
    for(size_t i = packet_id_sim_start; 
	i < packet_id_sim_end; i += packet_array_step)
    {
    	double t = section.m_packet_timestamps[i];

    	// pose, ray origin
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);

    	// packet pts
    	std::vector<std::vector<double> > this_real_pts = section.getPtsAtTime(t);

    	// add to big list of real pts
    	real_pts.insert(real_pts.end(), this_real_pts.begin(), this_real_pts.end());
	
	std::vector<double> this_ray_pitches;
	std::vector<double> this_ray_yaws;
	std::vector<std::vector<double> > this_real_pts_all;
	std::vector<int> this_real_hit_flag;
	std::tie(this_ray_pitches, this_ray_yaws, this_real_pts_all, this_real_hit_flag)
	    = ray_dirn_server.fitDetailToPts(ray_origin, this_real_pts);

    	// ray dirns
	std::vector<std::vector<double> > ray_dirns = calcRayDirnsFromSph(this_ray_pitches, this_ray_yaws);

    	// simulate 
    	std::vector<std::vector<double> > this_sim_pts_all;
    	std::vector<int> this_sim_hit_flag;
    	std::tie(this_sim_pts_all, this_sim_hit_flag) = sim.simPtsGivenRays(ray_origin, ray_dirns); 

    	// add to big list of sim pts
    	sim_pts_all.insert(sim_pts_all.end(), this_sim_pts_all.begin(), this_sim_pts_all.end());
    	hit_flag.insert(hit_flag.end(), this_sim_hit_flag.begin(), this_sim_hit_flag.end());

	// add to sim detail
	// ray origin
	sim_detail.m_ray_origins.push_back(ray_origin);
	// pitches
	sim_detail.m_ray_pitches.push_back(this_ray_pitches);
	// yaws
	sim_detail.m_ray_yaws.push_back(this_ray_yaws);
	// real pts all
	sim_detail.m_real_pts_all.push_back(this_real_pts_all);
	// real hit flag
	sim_detail.m_real_hit_flags.push_back(this_real_hit_flag);
	// sim pts all
	sim_detail.m_sim_pts_all.push_back(this_sim_pts_all);
	// sim hit flag
	sim_detail.m_sim_hit_flags.push_back(this_sim_hit_flag);
    }

    // weed out non-hits
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);

    // write real pts
    std::string rel_path_real_pts = genRelPathSliceRealPts(section_sim_id);
    writePtsToXYZFile(real_pts, rel_path_real_pts);
    
    // write sim detail
    std::string rel_path_sim_detail = genRelPathSimDetail(section_sim_id); 
    sim_detail.save(rel_path_sim_detail);

    // write sim pts
    std::string rel_path_sim_pts = genRelPathSimPts(section_sim_id);
    writePtsToXYZFile(sim_pts, rel_path_sim_pts);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
