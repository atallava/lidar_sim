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
#include <lidar_sim/RayDirnServer.h>

using namespace lidar_sim;

std::string genRelPathBlocksRealPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/blocks_real_pts.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_id = 3;
    std::string rel_path_section = genPathSection(section_id);
    SectionLoader section(rel_path_section);

    std::vector<std::vector<double> > real_pts;
    std::vector<int> non_ground_blocks {16,17,18,19};
    std::vector<int> ground_blocks {};
    
    // ground block pts
    std::vector<std::vector<double> > block_pts_all;
    for (auto block_id : ground_blocks)
    {
	std::string path_pts = genPathGroundBlockPts(section_id, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(path_pts);
	block_pts_all.insert(block_pts_all.begin(), 
			     block_pts.begin(), block_pts.end());
    }

    // non ground block pts
    for (auto block_id : non_ground_blocks)
    {
	std::string rel_path_pts = genPathNonGroundBlockPts(section_id, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);
	block_pts_all.insert(block_pts_all.begin(), 
			     block_pts.begin(), block_pts.end());
    }

    // pose server
    std::string path_poses_log = genPathPosesLog();
    PoseServer imu_pose_server(path_poses_log);

    // laser calib params
    LaserCalibParams laser_calib_params;

    RayDirnServer ray_dirn_server;

    // slice ids
    size_t packet_idx_sim_start, packet_idx_sim_end, packet_step;	
    packet_idx_sim_start = 0;
    packet_idx_sim_end = section.m_packet_timestamps.size()-1;
    packet_step = 10; 

    double hit_to_blocks_dist_threshold = 0.5;
    double resn_along_ray = 5;
    double miss_to_blocks_perp_threshold = 2;

    std::vector<std::vector<double> > slice_hits_in_block;
    FlannDatasetWrapper flann_dataset_wrapper(block_pts_all);
    std::vector<int> miss_rays_in_blocks;

    for (size_t i = packet_idx_sim_start; i <= packet_idx_sim_end; i += packet_step)
    {
	double t = section.m_packet_timestamps[i];
	
    	// pose, ray origin
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, laser_calib_params);

	// packet pts
    	std::vector<std::vector<double> > this_real_pts = section.getPtsAtTime(t);

	// ray dirns
	std::vector<double> this_ray_pitches;
	std::vector<double> this_ray_yaws;
	std::vector<std::vector<double> > this_real_pts_all;
	std::vector<int> this_real_hit_flag;
	std::tie(this_ray_pitches, this_ray_yaws, this_real_pts_all, this_real_hit_flag)
	    = ray_dirn_server.fitDetailToPts(ray_origin, this_real_pts);

	std::vector<std::vector<double> > ray_dirns = calcRayDirnsFromSph(this_ray_pitches, this_ray_yaws);

	// for each hit pt, check if within some threshold of block pts all, then store
	std::vector<std::vector<double> > nn_dists_to_blocks_pts;
	std::tie(std::ignore, nn_dists_to_blocks_pts) = flann_dataset_wrapper.knnSearch(this_real_pts_all, 1);
	size_t n_rays = ray_dirns.size();
	std::vector<int> hit_in_block_flag (n_rays, 0);
	for (size_t j = 0; j < n_rays; ++j)
	{
	    bool condn1 = this_real_hit_flag[j];
	    bool condn2 = (nn_dists_to_blocks_pts[j][0] < hit_to_blocks_dist_threshold);
	    if (condn1 && condn2) {
		hit_in_block_flag[j] = 1;
		slice_hits_in_block.push_back(this_real_pts_all[j]);
	    }
	}

	// todo: this could be smarter, by only querying rays which missed
	std::vector<double> ray_dists_to_blocks_pts;
	std::tie(std::ignore, ray_dists_to_blocks_pts) = 
	    flann_dataset_wrapper.approxNearestToRays(ray_origin, ray_dirns, laser_calib_params.intrinsics.max_range, resn_along_ray);
	int this_miss_rays_in_blocks = 0;
	for (size_t j = 0; j < n_rays; ++j)
	{
	    bool condn1 = (!this_real_hit_flag[j]);
	    bool condn2 = (ray_dists_to_blocks_pts[j] < miss_to_blocks_perp_threshold);
	    if (condn1 && condn2) 
		this_miss_rays_in_blocks++;
	}
	miss_rays_in_blocks.push_back(this_miss_rays_in_blocks); 
    }
    
    std::cout << "num slice hits in blocks: " << slice_hits_in_block.size() << std::endl;
    int total_miss_rays_in_blocks = 
	std::accumulate(miss_rays_in_blocks.begin(), miss_rays_in_blocks.end(), 0.0);
    std::cout << "total miss rays in blocks: " << total_miss_rays_in_blocks << std::endl;

    // write out
    std::string rel_path_pts = "data/misc/slice_hits_in_block.xyz";
    writePtsToXYZFile(slice_hits_in_block, rel_path_pts);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
