#include <algorithm>
#include <random>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/MeshModelSim.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

MeshModelSim::MeshModelSim() :
    m_max_dist_to_node_for_membership(100), 
    m_deterministic_sim(false),
    m_object_nn_radius(8)
{    
}

void MeshModelSim::loadObjectMeshes(const std::vector<std::string> &rel_path_object_meshes)
{
    for(size_t i = 0; i < rel_path_object_meshes.size(); ++i)
    {    
	TriangleModelSim sim;
	sim.loadTriangleModels(rel_path_object_meshes[i]);
	sim.setLaserCalibParams(m_laser_calib_params);
	sim.setDeterministicSim(m_deterministic_sim);

	m_object_mesh_sims.push_back(sim);
    }
    
    calcObjectCentroids();
}

void MeshModelSim::calcObjectCentroids()
{
    // check that meshes exist
    if (m_object_mesh_sims.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "MeshModelSim: m_object_mesh_sims is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    for(size_t i = 0; i < m_object_mesh_sims.size(); ++i)
    {
	std::vector<double> object_centroid = calcPtsMean(
	    m_object_mesh_sims[i].m_triangle_models.m_fit_pts);

	m_object_centroids.push_back(object_centroid);
    }
}

std::vector<int> MeshModelSim::calcObjectIdsForSim(const std::vector<double> &ray_origin, 
						   const std::vector<double> &ray_dirn)
{
    // dists along ray
    double max_range = m_laser_calib_params.intrinsics.max_range;
    std::vector<double> dists_along_ray;
    double walker = m_laser_calib_params.intrinsics.min_range;
    while (walker < max_range)
    {
	dists_along_ray.push_back(walker);
	walker += m_object_nn_radius;
    }
    dists_along_ray.push_back(max_range);
    
    // nodes along ray
    std::vector<std::vector<double> > nodes_along_ray;
    size_t n_nodes = dists_along_ray.size();
    for(size_t i = 0; i < n_nodes; ++i)
    {
	double dist_along_ray = dists_along_ray[i];
	std::vector<double> node(3,0);
	for(size_t j = 0; j < 3; ++j)
	    node[j] = ray_origin[j] + dist_along_ray*ray_dirn[j];

	nodes_along_ray.push_back(node);
    }
    
    // nn for each node
    std::vector<std::vector<int> > nn_ids;
    std::vector<std::vector<double> > nn_dists;
    int n_objects = m_object_mesh_sims.size();
    int num_nbrs = std::min(20, n_objects);
    std::tie(nn_ids, nn_dists) = nearestNeighbors(m_object_centroids, nodes_along_ray, num_nbrs);
    

    // object ids
    std::vector<int> object_ids;
    for(size_t i = 0; i < n_nodes; ++i)
	for(size_t j = 0; j < (size_t)num_nbrs; ++j)
	    if (nn_dists[i][j] <= m_object_nn_radius)
		object_ids.push_back(nn_ids[i][j]);
    // retain unique ids
    object_ids = getUniqueSortedVec(object_ids);

    // todo: delete
    // debug
    // std::cout << "dists along ray" << std::endl;
    // dispVec(dists_along_ray);
    // std::cout << "nodes along ray" << std::endl;
    // dispMat(nodes_along_ray);
    // std::cout << "nn ids" << std::endl;
    // dispMat(nn_ids);
    // std::cout << "nn dists" << std::endl;
    // dispMat(nn_dists);
    // std::cout << "obj ids" << std::endl;
    // dispVec(object_ids);

    return object_ids;
}

std::vector<int> MeshModelSim::calcObjectIdsForSim(const std::vector<double> &ray_origin, 
						   const std::vector<std::vector<double> > &ray_dirns)
{
    std::vector<int> object_ids;
    for(size_t i = 0; i < ray_dirns.size(); ++i)
    {
	std::vector<double> ray_dirn = ray_dirns[i];
	std::vector<int> ray_object_ids = 
	    calcObjectIdsForSim(ray_origin, ray_dirn);

	object_ids.insert(object_ids.end(), 
			  ray_object_ids.begin(), ray_object_ids.end());
    }
    object_ids = getUniqueSortedVec(object_ids);
    
    return object_ids;
}

// load triangle model blocks
void MeshModelSim::loadTriangleModelBlocks(const std::vector<std::string> &rel_path_model_blocks)
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
void MeshModelSim::loadBlockInfo(const std::string rel_path_imu_posn_nodes, 
				 const std::string rel_path_block_node_ids_ground)
{
    m_imu_posn_nodes = loadArray(rel_path_imu_posn_nodes, 3);
    m_block_node_ids_ground = doubleToIntArray(loadArray(rel_path_block_node_ids_ground, 2));
}

// triangle block membership
std::vector<int> MeshModelSim::getPosnTriangleBlockMembership(const std::vector<double> &posn)
{
    return getPosnBlockMembership(posn, m_block_node_ids_ground);
}

std::vector<int> MeshModelSim::getPoseBlockMembership(const std::vector<double> &imu_pose, 
							 const std::vector<std::vector<int> > &block_node_ids)
{
    std::vector<double> imu_posn = posnFromImuPose(imu_pose);
    return getPosnBlockMembership(imu_posn, block_node_ids);
}

std::vector<int> MeshModelSim::getPosnBlockMembership(const std::vector<double> &posn, 
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
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> >
MeshModelSim::simPtsGivenPose(const std::vector<double> &imu_pose)
{
    // rays
    std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, m_laser_calib_params);
    std::vector<std::vector<double> > ray_dirns = genRayDirnsWorldFrame(imu_pose, m_laser_calib_params);

    return simPtsGivenRays(ray_origin, ray_dirns);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
MeshModelSim::simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses)
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
MeshModelSim::simPtsGivenRays(const std::vector<double> &ray_origin, 
				 const std::vector<std::vector<double> > &ray_dirns)
{
    typedef std::vector<std::vector<double> > SimPts; 
    typedef std::vector<int> HitFlag;

    // will continue calling blocks
    // even though some 'blocks' are objects
    std::vector<SimPts> sim_pts_over_blocks;
    std::vector<HitFlag> hit_flag_over_blocks;
   
    // sim from objects
    std::vector<int> object_ids_to_sim = calcObjectIdsForSim(ray_origin, ray_dirns);
    std::vector<int> objects_hit;
    for(size_t i = 0; i < object_ids_to_sim.size(); ++i)
    {
    	std::vector<std::vector<double> > sim_pts_can;
    	std::vector<int> hit_flag_can;
	int object_id = object_ids_to_sim[i];
	
    	std::tie(sim_pts_can, hit_flag_can) = 
    	    m_object_mesh_sims[object_id].simPtsGivenRays(ray_origin, ray_dirns);
    	if (anyNonzeros(hit_flag_can))
    	    objects_hit.push_back(object_id);
	
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

    // todo: comment/ delete
    // std::cout << "MeshModelSim: objects hit: " << std::endl;
    // dispVec(objects_hit);
    // std::cout << "MeshModelSim: triangle blocks hit: " << std::endl;
    // dispVec(triangle_blocks_hit);
    
    return std::make_tuple(sim_pts, hit_flag);
}

void MeshModelSim::setDeterministicSim(const bool choice)
{
    m_deterministic_sim = choice;

    // object mesh sims
    for(size_t i = 0; i < m_object_mesh_sims.size(); ++i)
	m_object_mesh_sims[i].setDeterministicSim(m_deterministic_sim);

    // triangle model sims
    if (m_triangle_model_sims.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "MeshModelSim: m_triangle_model_sims is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    for(size_t i = 0; i < m_triangle_model_sims.size(); ++i)
	m_triangle_model_sims[i].setDeterministicSim(m_deterministic_sim);
}
