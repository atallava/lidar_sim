#include <algorithm>
#include <random>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/SectionModelSim.h>

using namespace lidar_sim;

SectionModelSim::SectionModelSim() :
    m_max_dist_to_node_for_membership(20)
{    
}

// load ellipsoid model blocks
void SectionModelSim::loadEllipsoidModelBlocks(const std::vector<std::string> &rel_path_model_blocks)
{
    for(size_t i = 0; i < rel_path_model_blocks.size(); ++i)
    {    
	EllipsoidModelSim sim;
	sim.loadEllipsoidModels(rel_path_model_blocks[i]);
	sim.setLaserCalibParams(m_laser_calib_params);

	m_ellipsoid_model_sims.push_back(sim);
    }
}

// load triangle model blocks
void SectionModelSim::loadTriangleModelBlocks(const std::vector<std::string> &rel_path_model_blocks)
{
    for(size_t i = 0; i < rel_path_model_blocks.size(); ++i)
    {    
	TriangleModelSim sim;
	sim.loadTriangleModels(rel_path_model_blocks[i]);
	sim.setLaserCalibParams(m_laser_calib_params);

	m_triangle_model_sims.push_back(sim);
    }
}

// load block info
void SectionModelSim::loadBlockInfo(const std::string rel_path_imu_posn_nodes, const std::string rel_path_block_node_ids_ground, const std::string rel_path_block_node_ids_non_ground)
{
    m_imu_posn_nodes = loadArray(rel_path_imu_posn_nodes, 3);
    m_block_node_ids_ground = doubleToIntArray(loadArray(rel_path_block_node_ids_ground, 2));
    m_block_node_ids_non_ground = doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));
}

// ellipsoid block membership
std::vector<int> SectionModelSim::getPoseEllipsoidBlockMembership(const std::vector<double> &imu_pose)
{
    return getPoseBlockMembership(imu_pose, m_block_node_ids_non_ground);
}

// triangle block membership
std::vector<int> SectionModelSim::getPoseTriangleBlockMembership(const std::vector<double> &imu_pose)
{
    return getPoseBlockMembership(imu_pose, m_block_node_ids_ground);
}

std::vector<int> SectionModelSim::getPoseBlockMembership(const std::vector<double> &imu_pose, const std::vector<std::vector<int> > &block_node_ids)
{
    // distances to ellipsoid block boundary nodes
    std::vector<std::vector<double> > dists_to_nodes(block_node_ids.size(), std::vector<double>(2));
    std::vector<int> block_ids_belonging_to;
    std::vector<double> imu_posn = posnFromImuPose(imu_pose);
    for(size_t i = 0; i < block_node_ids.size(); ++i)
    {
	for(size_t j = 0; j < 2; ++j)
	    dists_to_nodes[i][j] = euclideanDist(imu_posn, m_imu_posn_nodes[block_node_ids[i][j]]);
	
	if ((dists_to_nodes[i][0] <= m_max_dist_to_node_for_membership) || 
	    (dists_to_nodes[i][1] <= m_max_dist_to_node_for_membership))
	    block_ids_belonging_to.push_back(i);
    }

    return block_ids_belonging_to;
}

// sim pt given pose
std::tuple<std::vector<std::vector<double> >, std::vector<int> >
SectionModelSim::simPtsGivenPose(const std::vector<double> &imu_pose)
{
    typedef std::vector<std::vector<double> > SimPts; // is this expensive?
    typedef std::vector<int> HitFlag;

    std::vector<SimPts> sim_pts_over_blocks;
    std::vector<HitFlag> hit_flag_over_blocks;

    // sim over ellipsoid blocks
    std::vector<int> ellipsoid_blocks_to_sim = getPoseBlockMembership(imu_pose, m_block_node_ids_non_ground);
    std::vector<int> ellipsoid_blocks_hit;
    for(size_t i = 0; i < ellipsoid_blocks_to_sim.size(); ++i)
    {
	std::vector<std::vector<double> > sim_pts_can;
	std::vector<int> hit_flag_can;
	int block_id = ellipsoid_blocks_to_sim[i];
	try
	{	
	    std::tie(sim_pts_can, hit_flag_can) = 
		m_ellipsoid_model_sims[block_id].simPtsGivenPose(imu_pose);
	}
	catch (const std::exception& e)
	{
	    std::cout << "error" << std::endl;
	    std::cout << block_id << std::endl;
	    exit(0);
	}
	if (anyNonzeros(hit_flag_can))
	    ellipsoid_blocks_hit.push_back(block_id);
	
	sim_pts_over_blocks.push_back(sim_pts_can);
	hit_flag_over_blocks.push_back(hit_flag_can);
    }

    // sim over tri blocks
    std::vector<int> triangle_blocks_to_sim = getPoseBlockMembership(imu_pose, m_block_node_ids_ground);
    std::vector<int> triangle_blocks_hit;
    for(size_t i = 0; i < triangle_blocks_to_sim.size(); ++i)
    {
    	std::vector<std::vector<double> > sim_pts_can;
    	std::vector<int> hit_flag_can;
    	int block_id = triangle_blocks_to_sim[i];
    	std::tie(sim_pts_can, hit_flag_can) = 
    	    m_triangle_model_sims[block_id].simPtsGivenPose(imu_pose);
    	if (anyNonzeros(hit_flag_can))
    	    triangle_blocks_hit.push_back(block_id);
	
    	sim_pts_over_blocks.push_back(sim_pts_can);
    	hit_flag_over_blocks.push_back(hit_flag_can);
    }

    // marginalize 
    int n_rays = m_laser_calib_params.intrinsics.getNRays();
    std::vector<std::vector<double> > sim_pts(n_rays, std::vector<double> (3,0));
    std::vector<int> hit_flag(n_rays, 0);

    std::vector<double> laser_posn = laserPosnFromImuPose(imu_pose, m_laser_calib_params);
    // loop over rays
    for(size_t i = 0; i < (size_t)n_rays; ++i)
    {
	std::vector<double> ranges_over_blocks(sim_pts_over_blocks.size(), 1e7); // just some big value
	int misses = 0;
	// loop over each block
	for(size_t j = 0; j < sim_pts_over_blocks.size(); ++j)
	    if (hit_flag_over_blocks[j][i])
		ranges_over_blocks[j] = euclideanDist(laser_posn, sim_pts_over_blocks[j][i]);
	    else
		misses += 1;

	// if hit a block
	// take closest range value
	if (misses < (int)sim_pts_over_blocks.size())
	{
	    auto min_it = std::min_element(std::begin(ranges_over_blocks), std::end(ranges_over_blocks));
	    int min_posn = std::distance(std::begin(ranges_over_blocks), min_it);
	    sim_pts[i] = sim_pts_over_blocks[min_posn][i];
	    hit_flag[i] = 1;
	}
    }
    
    return std::make_tuple(sim_pts, hit_flag);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
SectionModelSim::simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses)
{
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    for(size_t i = 0; i < imu_poses.size(); ++i)
    {
	std::vector<std::vector<double> > this_sim_pts;
	std::vector<int> this_hit_flag;
	std::tie(this_sim_pts, this_hit_flag) = simPtsGivenPose(imu_poses[i]); 
	for(size_t j = 0; j < this_sim_pts.size(); ++j)
	{
	    sim_pts.push_back(this_sim_pts[j]);
	    hit_flag.push_back(this_hit_flag[j]);
	}
    }

    return std::make_tuple(sim_pts, hit_flag);
}
