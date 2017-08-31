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
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionModelSim.h>
#include <lidar_sim/SimDetail.h>
#include <lidar_sim/RayDirnServer.h>

using namespace lidar_sim;

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathEllipsoids(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

std::string genRelPathBlocksRealPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/blocks_real_pts.xyz";

    return ss.str();
}

std::string genRelPathSimPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/blocks_sim_pts.xyz";

    return ss.str();
}

std::string genRelPathSimDetail(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/blocks_sim_detail.txt";

    return ss.str();
}

std::string genRelPathModelsDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/hg_sim";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_id_for_sim = 3;
    std::string rel_path_section = genRelPathSection(section_id_for_sim);
    SectionLoader section(rel_path_section);

    // sim object
    int section_models_id = 3;
    std::string rel_path_models_dir = genRelPathModelsDir(section_models_id);;

    // find ellipsoid models for this section
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    std::vector<int> ellipsoid_model_block_ids = 
	getEllipsoidModelBlockIds(rel_path_models_dir, section_models_id);
    for(auto i : ellipsoid_model_block_ids)
    	rel_path_ellipsoid_model_blocks.push_back(genRelPathEllipsoids(section_models_id, i));

    // find triangle models for this section
    std::vector<std::string> rel_path_triangle_model_blocks;
    std::vector<int> triangle_model_block_ids = 
	getTriangleModelBlockIds(rel_path_models_dir, section_models_id);
    for(auto i : triangle_model_block_ids)
    	rel_path_triangle_model_blocks.push_back(genRelPathTriangles(section_models_id, i));

    // create sim object
    SectionModelSim sim;
    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
    sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);

    sim.setDeterministicSim(false);

    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_models_id);
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(section_models_id);
    std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_models_id);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // get section pt ids to sim
    std::vector<int> non_ground_blocks_for_sim {16,17,18,19};
    std::vector<int> ground_blocks_for_sim {};
    std::vector<int> section_pt_ids_for_sim;
    int num_nbrs = 1;
    // ground block pts
    for (auto block_id : ground_blocks_for_sim)
    {
	std::string rel_path_pts = genRelPathGroundBlockPts(section_id_for_sim, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);

	std::vector<std::vector<int> > section_nbr_pt_ids; 

	std::tie(section_nbr_pt_ids, std::ignore) = 
	    nearestNeighbors(section.m_pts, block_pts, num_nbrs);

	for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	    for(size_t j = 0; j < (size_t)num_nbrs; ++j)
	    {
		int idx = section_nbr_pt_ids[i][j];
		section_pt_ids_for_sim.push_back(idx);
	    }
    }

    // non ground block pts
    for (auto block_id : non_ground_blocks_for_sim)
    {
	std::string rel_path_pts = genRelPathNonGroundBlockPts(section_id_for_sim, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);

	std::vector<std::vector<int> > section_nbr_pt_ids; 

	std::tie(section_nbr_pt_ids, std::ignore) = 
	    nearestNeighbors(section.m_pts, block_pts, num_nbrs);

	for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	    for(size_t j = 0; j < (size_t)num_nbrs; ++j)
	    {
		int idx = section_nbr_pt_ids[i][j];
		section_pt_ids_for_sim.push_back(idx);
	    }
    }

    // subsample pt ids for sim
    std::vector<int> section_pt_ids_subsampled;
    int max_pt_ids_for_sim = 1e4;
    int step = std::floor(section_pt_ids_for_sim.size()/max_pt_ids_for_sim);
    if (step < 1)
	step = 1;
    for (size_t i = 0; i < section_pt_ids_for_sim.size(); i += step)
	section_pt_ids_subsampled.push_back(
	    section_pt_ids_for_sim[i]);
    section_pt_ids_for_sim = section_pt_ids_subsampled;

    // sim
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> sim_hit_flag;
    std::vector<std::vector<double> > real_pts;
    SimDetail sim_detail;
    for(auto idx : section_pt_ids_for_sim)
    {
    	double t = section.m_pt_timestamps[idx];

    	// pose, ray origin, ray dirn
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);
	std::vector<double> this_real_pt = section.m_pts[idx];
	std::vector<double> ray_dirn;
	std::tie(ray_dirn, std::ignore) = calcRayDirn(ray_origin, this_real_pt);

	// add to real pts
	real_pts.push_back(this_real_pt);

	// variables needed by sim detail
	double this_yaw, this_pitch;
	std::tie(this_yaw, this_pitch, std::ignore) = cart2sph(ray_dirn);
	std::vector<double> this_ray_pitches {this_pitch};
	std::vector<double> this_ray_yaws {this_yaw};
	std::vector<std::vector<double> > this_real_pts_all = wrapDataInVec(this_real_pt);
	std::vector<int> this_real_hit_flag {1};

	// wrapping for conformity
	std::vector<std::vector<double> > ray_dirns = wrapDataInVec(ray_dirn);

    	// simulate 
    	std::vector<std::vector<double> > this_sim_pts_all;
    	std::vector<int> this_sim_hit_flag;
    	std::tie(this_sim_pts_all, this_sim_hit_flag) = sim.simPtsGivenRays(ray_origin, ray_dirns); 

    	// add to big list of sim pts
    	sim_pts_all.insert(sim_pts_all.end(), this_sim_pts_all.begin(), this_sim_pts_all.end());
    	sim_hit_flag.insert(sim_hit_flag.end(), this_sim_hit_flag.begin(), this_sim_hit_flag.end());

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
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, sim_hit_flag);

    // write real pts
    std::string rel_path_real_pts = genRelPathBlocksRealPts(section_id_for_sim);
    writePtsToXYZFile(real_pts, rel_path_real_pts);

    // write sim pts
    std::string rel_path_sim_pts = genRelPathSimPts(section_id_for_sim);
    writePtsToXYZFile(sim_pts, rel_path_sim_pts);

    // write sim detail
    std::string rel_path_sim_detail = genRelPathSimDetail(section_id_for_sim); 
    sim_detail.save(rel_path_sim_detail);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
