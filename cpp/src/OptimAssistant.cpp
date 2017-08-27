#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <lidar_sim/OptimAssistant.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/EllipsoidModeler.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionModelSim.h>
#include <lidar_sim/RayDirnServer.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/SimDetail.h>

using namespace lidar_sim;

OptimAssistant::OptimAssistant() :
    m_verbose(false),
    m_initialized(false),
    m_section_packet_step(1),
    m_num_nbrs_for_hit_prob(1)
{
    m_rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    m_imu_pose_server = PoseServer(m_rel_path_poses_log);
}

double OptimAssistant::calcObj(std::vector<double> x)
{
    if (!m_initialized)
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "OptimAssistant has not been initialized.";
	// todo: what is the right error to throw here?
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    // create the ellipsoid block models
    for (auto block_id : m_non_ground_block_ids)
	buildModelsNonGroundBlock(block_id, x);

    // simulate
    simulate();

    // sim section packet ids
    // calc and return error
    return x[0];
}

void OptimAssistant::init()
{
    // todo: how do you know section id for models has been set?
    std::string m_rel_path_section_for_model = 
	genRelPathSection(m_section_id_for_model);
    m_section_for_model = SectionLoader (m_rel_path_section_for_model);

    std::string m_rel_path_section_for_sim = 
	genRelPathSection(m_section_id_for_sim);
    m_section_for_sim = SectionLoader (m_rel_path_section_for_sim);

    m_initialized = true;
}

void OptimAssistant::buildModelsNonGroundBlock(const int block_id, const std::vector<double> x)
{
    std::string rel_path_pts = genRelPathBlockPts(m_section_id_for_model, block_id);
    std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts, m_verbose);
    
    EllipsoidModeler modeler;
    modeler.setVerbosity(m_verbose);

    // parameters fed here
    modeler.m_n_clusters_per_pt = x[0];
    modeler.m_set_max_maha_dist_for_hit = true;
    modeler.m_max_maha_dist_for_hit = x[1];

    modeler.createEllipsoidModels(rel_path_pts);

    // hit prob calculation
    std::vector<std::vector<int> > section_nbr_pt_ids; 

    std::tie(section_nbr_pt_ids, std::ignore) = 
	nearestNeighbors(m_section_for_model.m_pts, block_pts, m_num_nbrs_for_hit_prob);

    std::vector<int> section_pt_ids_to_process;
    for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	for(size_t j = 0; j < (size_t)m_num_nbrs_for_hit_prob; ++j)
	{
	    int id = section_nbr_pt_ids[i][j];
	    section_pt_ids_to_process.push_back(id);
	}
    modeler.calcHitProb(m_section_for_model, section_pt_ids_to_process, m_imu_pose_server);

    // write ellipsoids
    std::string rel_path_ellipsoids = genRelPathEllipsoids(m_section_id_for_model, block_id);
    modeler.writeEllipsoidsToFile(rel_path_ellipsoids);
}

void OptimAssistant::simulate()
{
    // ellipsoid model paths
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    for (auto block_id : m_non_ground_block_ids)
	rel_path_ellipsoid_model_blocks.push_back(
	    genRelPathEllipsoids(m_section_id_for_model, block_id));

    // triangle model paths
    std::vector<std::string> rel_path_triangle_model_blocks;
    for (auto block_id : m_ground_block_ids)
	rel_path_triangle_model_blocks.push_back(
	    genRelPathTriangles(m_section_id_for_model, block_id));

    // create sim object
    SectionModelSim sim;
    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
    sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);
    sim.setDeterministicSim(false);

    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(m_section_id_for_model);
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(m_section_id_for_model);
    std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(m_section_id_for_model);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);
    RayDirnServer ray_dirn_server;
    
      // sim
    // loop over packets
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> sim_hit_flag;
    std::vector<std::vector<double> > real_pts;
    SimDetail sim_detail;
    sim_detail.setVerbosity(m_verbose);
    for(size_t i = m_section_packet_start; 
	i < (size_t)m_section_packet_end; i += m_section_packet_step)
    {
    	double t = m_section_for_sim.m_packet_timestamps[i];

    	// pose, ray origin
    	std::vector<double> imu_pose = m_imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);

	// packet pts
    	std::vector<std::vector<double> > this_real_pts = m_section_for_sim.getPtsAtTime(t);

    	// add to big list of real pts
    	real_pts.insert(real_pts.end(), this_real_pts.begin(), this_real_pts.end());

    	// ray dirns
	std::vector<double> this_ray_pitches;
	std::vector<double> this_ray_yaws;
	std::vector<std::vector<double> > this_real_pts_all;
	std::vector<int> this_real_hit_flag;
	std::tie(this_ray_pitches, this_ray_yaws, this_real_pts_all, this_real_hit_flag)
	    = ray_dirn_server.fitDetailToPts(ray_origin, this_real_pts);

	std::vector<std::vector<double> > ray_dirns = calcRayDirnsFromSph(this_ray_pitches, this_ray_yaws);
	
    	// simulate 
    	std::vector<std::vector<double> > this_sim_pts_all;
    	std::vector<int> this_sim_hit_flag;
    	std::tie(this_sim_pts_all, this_sim_hit_flag) = sim.simPtsGivenRays(ray_origin, ray_dirns); 

    	// add to big list of sim pts
    	sim_pts_all.insert(sim_pts_all.end(), this_sim_pts_all.begin(), this_sim_pts_all.end());
    	sim_hit_flag.insert(sim_hit_flag.end(), this_sim_hit_flag.begin(), this_sim_hit_flag.end());

	// add to sim detail
	// todo: this should be cleaner?
	sim_detail.m_ray_origins.push_back(ray_origin);
	sim_detail.m_ray_pitches.push_back(this_ray_pitches);
	sim_detail.m_ray_yaws.push_back(this_ray_yaws);
	sim_detail.m_real_pts_all.push_back(this_real_pts_all);
	sim_detail.m_real_hit_flags.push_back(this_real_hit_flag);
	sim_detail.m_sim_pts_all.push_back(this_sim_pts_all);
	sim_detail.m_sim_hit_flags.push_back(this_sim_hit_flag);
    }

    // weed out non-hit sim pts
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, sim_hit_flag);

    // write real pts
    std::string rel_path_real_pts = genRelPathSliceRealPts(m_section_id_for_sim);
    writePtsToXYZFile(real_pts, rel_path_real_pts, m_verbose);

    // write sim pts
    std::string rel_path_sim_pts = genRelPathSimPts(m_section_id_for_sim);
    writePtsToXYZFile(sim_pts, rel_path_sim_pts, m_verbose);

    // write sim detail
    std::string rel_path_sim_detail = genRelPathSimDetail(m_section_id_for_sim); 
    sim_detail.save(rel_path_sim_detail);  
}

std::string OptimAssistant::genRelPathBlockPts(const int section_id, const int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

    return ss.str();
}

std::string OptimAssistant::genRelPathSection(const int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_subsampled.xyz";

    return ss.str();
}

std::string OptimAssistant::genRelPathEllipsoids(const int section_id, const int block_id)
{
    std::ostringstream ss;
    ss << "data/sim_optim/models" 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sim_optim/models"
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

// todo: i'd like these to be somewhere else?  notice how some of the rel paths
// go into data/sections, but other go into data/sim_optim
std::string OptimAssistant::genRelPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathBlockNodeIdsGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/block_node_ids_ground.txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathBlockNodeIdsNonGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/block_node_ids_non_ground.txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathSliceRealPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sim_optim/sim/"
       << "section_" << std::setw(2) << std::setfill('0') << section_id
       << "_slice_real_pts.xyz";

    return ss.str();
}

std::string OptimAssistant::genRelPathSimPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sim_optim/sim/"
       << "section_" << std::setw(2) << std::setfill('0') << section_id
       << "_slice_sim_pts.xyz";

    return ss.str();
}

std::string OptimAssistant::genRelPathSimDetail(int section_id)
{
    std::ostringstream ss;
    ss << "data/sim_optim/sim/"
       << "section_" << std::setw(2) << std::setfill('0') << section_id
       << "_slice_sim_detail.txt";

    return ss.str();
}

