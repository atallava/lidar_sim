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

using namespace lidar_sim;

OptimAssistant::OptimAssistant() :
    m_verbose(false),
    m_initialized(false),
    m_section_packet_skip(1),
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

    // create the ellipsoid block models!
    for (auto block_id : m_non_ground_block_ids)
	buildModelsNonGroundBlock(block_id, x);

    // sim section packet ids
    // calc and return error
    return x[0];
}

void OptimAssistant::init()
{
    // todo: how do you know section id for models has been set?
    std::string m_rel_path_section_for_model = 
	genRelPathSection(m_section_id_for_model);
    m_section_for_model = SectionLoader(m_rel_path_section_for_model);

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


