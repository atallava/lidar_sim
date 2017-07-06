#include <algorithm>
#include <random>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/SectionModelSim.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

SectionModelSim::SectionModelSim() :
    m_max_dist_to_node_for_membership(100), 
    m_deterministic_sim(false)
{    
}

// load ellipsoid model blocks
void SectionModelSim::loadEllipsoidModelBlocks(const std::vector<std::string> &rel_path_model_blocks)
{
    std::vector<EllipsoidModel> ellipsoid_models_all;
    for(size_t i = 0; i < rel_path_model_blocks.size(); ++i)
    {    
	EllipsoidModelSim sim;
	sim.loadEllipsoidModels(rel_path_model_blocks[i]);
	sim.setLaserCalibParams(m_laser_calib_params);
	sim.setDeterministicSim(m_deterministic_sim);

	ellipsoid_models_all.insert(ellipsoid_models_all.end(),
				    sim.m_ellipsoid_models.begin(), sim.m_ellipsoid_models.end());

	m_ellipsoid_model_sims.push_back(sim);
    }

    m_ellipsoid_sim_nbr_server.setEllipsoidModels(ellipsoid_models_all);
}

// load triangle model blocks
void SectionModelSim::loadTriangleModelBlocks(const std::vector<std::string> &rel_path_model_blocks)
{
    for(size_t i = 0; i < rel_path_model_blocks.size(); ++i)
    {    
	TriangleModelSim sim;
	sim.loadTriangleModels(rel_path_model_blocks[i]);
	sim.setLaserCalibParams(m_laser_calib_params);
	sim.setDeterministicSim(m_deterministic_sim);

	m_triangle_model_sims.push_back(sim);
    }
}

// load block info
void SectionModelSim::loadBlockInfo(const std::string rel_path_imu_posn_nodes, const std::string rel_path_block_node_ids_ground, 
				    const std::string rel_path_block_node_ids_non_ground)
{
    m_imu_posn_nodes = loadArray(rel_path_imu_posn_nodes, 3);
    m_block_node_ids_ground = doubleToIntArray(loadArray(rel_path_block_node_ids_ground, 2));
    m_block_node_ids_non_ground = doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));
}

// ellipsoid block membership
std::vector<int> SectionModelSim::getPosnEllipsoidBlockMembership(const std::vector<double> &posn)
{
    return getPosnBlockMembership(posn, m_block_node_ids_non_ground);
}

// triangle block membership
std::vector<int> SectionModelSim::getPosnTriangleBlockMembership(const std::vector<double> &posn)
{
    return getPosnBlockMembership(posn, m_block_node_ids_ground);
}

std::vector<int> SectionModelSim::getPoseBlockMembership(const std::vector<double> &imu_pose, 
							 const std::vector<std::vector<int> > &block_node_ids)
{
    std::vector<double> imu_posn = posnFromImuPose(imu_pose);
    return getPosnBlockMembership(imu_posn, block_node_ids);
}

std::vector<int> SectionModelSim::getPosnBlockMembership(const std::vector<double> &posn, 
							 const std::vector<std::vector<int> > &block_node_ids)
{
    // distances to imu posn nodes
    std::vector<double> dists_to_imu_posn_nodes;
    std::vector<int> flag;
    for(size_t i = 0; i < m_imu_posn_nodes.size(); ++i)
    {
	auto begin = m_imu_posn_nodes[i].begin();
	std::vector<double> node_posn(begin, begin + 3);

	double dist_to_node = euclideanDist(posn, node_posn);
	dists_to_imu_posn_nodes.push_back(dist_to_node);
	if (dist_to_node > m_max_dist_to_node_for_membership)
	    flag.push_back(0);
	else
	    flag.push_back(1);
    }

    // find which blocks have been activated
    std::vector<int> block_ids_belonging_to;
    for(size_t i = 0; i < block_node_ids.size(); ++i)
    {
	// if any of the flagged nodes belongs to this block, activate
	for(size_t j = 0; j < flag.size(); ++j)
	    if (flag[j])
	    {
		bool condn1 = (block_node_ids[i][0] <= (int)j);
		bool condn2 = ((int)j <= block_node_ids[i][1]);
		bool condn3 = (condn1) && (condn2);
		if (condn3)
		{
		    // note: push back i+1 because blocks begin indexing from 1, not 0
		    block_ids_belonging_to.push_back(i+1);
		    break; // since already activated
		}
	    }
    }

    return block_ids_belonging_to;

    // // distances to block boundary nodes
    // std::vector<std::vector<double> > dists_to_nodes(block_node_ids.size(), std::vector<double>(2));
    // std::vector<int> block_ids_belonging_to;
    // for(size_t i = 0; i < block_node_ids.size(); ++i)
    // {
    // 	for(size_t j = 0; j < 2; ++j)
    // 	    dists_to_nodes[i][j] = euclideanDist(posn, m_imu_posn_nodes[block_node_ids[i][j]]);
	
    // 	if ((dists_to_nodes[i][0] <= m_max_dist_to_node_for_membership) || 
    // 	    (dists_to_nodes[i][1] <= m_max_dist_to_node_for_membership))
    // 	    // warning: push back i+1 because blocks begin indexing from 1, not 0
    // 	    block_ids_belonging_to.push_back(i+1);
    // }

    // return block_ids_belonging_to;
}

// sim pt given pose
std::tuple<std::vector<std::vector<double> >, std::vector<int> >
SectionModelSim::simPtsGivenPose(const std::vector<double> &imu_pose)
{
    // rays
    std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, m_laser_calib_params);
    std::vector<std::vector<double> > ray_dirns = genRayDirnsWorldFrame(imu_pose, m_laser_calib_params);

    return simPtsGivenRays(ray_origin, ray_dirns);
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

std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
SectionModelSim::simPtsGivenRays(const std::vector<double> &ray_origin, 
				 const std::vector<std::vector<double> > &ray_dirns)
{
    typedef std::vector<std::vector<double> > SimPts; 
    typedef std::vector<int> HitFlag;
    
    std::vector<SimPts> sim_pts_over_blocks;
    std::vector<HitFlag> hit_flag_over_blocks;

    // todo: clean this up
    int ellipsoid_choice = 1;

    if (ellipsoid_choice == 0)
    {
	// sim over ellipsoid blocks
	std::vector<int> ellipsoid_blocks_to_sim = getPosnBlockMembership(ray_origin, m_block_node_ids_non_ground);
	std::vector<int> ellipsoid_blocks_hit;
	for(size_t i = 0; i < ellipsoid_blocks_to_sim.size(); ++i)
	{
	    std::vector<std::vector<double> > sim_pts_can;
	    std::vector<int> hit_flag_can;
	    int block_id = ellipsoid_blocks_to_sim[i];
	    try
	    {	
		// block_id - 1, since blocks are indexed starting 1
		std::tie(sim_pts_can, hit_flag_can) = 
		    m_ellipsoid_model_sims[block_id - 1].simPtsGivenRays(ray_origin, ray_dirns);
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
    }
    else
    {
	// one 'block' will just be the local ellipsoid model sim
	EllipsoidModelSim ellipsoid_sim_nbr = m_ellipsoid_sim_nbr_server.createSim(ray_origin, ray_dirns);
	ellipsoid_sim_nbr.setDeterministicSim(m_deterministic_sim);
	std::vector<std::vector<double> > sim_pts_can;
	std::vector<int> hit_flag_can;
	std::tie(sim_pts_can, hit_flag_can) = ellipsoid_sim_nbr.simPtsGivenRays(ray_origin, ray_dirns);
	sim_pts_over_blocks.push_back(sim_pts_can);
	hit_flag_over_blocks.push_back(hit_flag_can);
    }

    // sim over tri blocks
    std::vector<int> triangle_blocks_to_sim = getPosnBlockMembership(ray_origin, m_block_node_ids_ground);
    std::vector<int> triangle_blocks_hit;
    for(size_t i = 0; i < triangle_blocks_to_sim.size(); ++i)
    {
    	std::vector<std::vector<double> > sim_pts_can;
    	std::vector<int> hit_flag_can;
    	int block_id = triangle_blocks_to_sim[i];
    	// block_id - 1, since blocks are indexed starting 1
    	std::tie(sim_pts_can, hit_flag_can) = 
    	    m_triangle_model_sims[block_id - 1].simPtsGivenRays(ray_origin, ray_dirns);
    	if (anyNonzeros(hit_flag_can))
    	    triangle_blocks_hit.push_back(block_id);
	
    	sim_pts_over_blocks.push_back(sim_pts_can);
    	hit_flag_over_blocks.push_back(hit_flag_can);
    }

    // marginalize 
    int n_rays = ray_dirns.size();
    std::vector<std::vector<double> > sim_pts(n_rays, std::vector<double> (3,0));
    std::vector<int> hit_flag(n_rays, 0);

    // loop over rays
    for(size_t i = 0; i < (size_t)n_rays; ++i)
    {
	std::vector<double> ranges_over_blocks(sim_pts_over_blocks.size(), 1e7); // just some big value
	int misses = 0;
	// loop over each block
	for(size_t j = 0; j < sim_pts_over_blocks.size(); ++j)
	    if (hit_flag_over_blocks[j][i])
		ranges_over_blocks[j] = euclideanDist(ray_origin, sim_pts_over_blocks[j][i]);
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

    // debug
    // std::cout << "SectionModelSim: ellipsoid blocks hit: " << std::endl;
    // dispVec(ellipsoid_blocks_hit);
    // std::cout << "SectionModelSim: triangle blocks hit: " << std::endl;
    // dispVec(triangle_blocks_hit);
    
    return std::make_tuple(sim_pts, hit_flag);
}

void SectionModelSim::setDeterministicSim(const bool choice)
{
    m_deterministic_sim = choice;

    // triangle model sims
    if (m_triangle_model_sims.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "SectionModelSim: m_triangle_model_sims is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    for(size_t i = 0; i < m_triangle_model_sims.size(); ++i)
	m_triangle_model_sims[i].setDeterministicSim(m_deterministic_sim);

    // ellipsoid model sims
    if (m_ellipsoid_model_sims.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "SectionModelSim: m_ellipsoid_model_sims is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    for(size_t i = 0; i < m_ellipsoid_model_sims.size(); ++i)
	m_ellipsoid_model_sims[i].setDeterministicSim(m_deterministic_sim);
}

