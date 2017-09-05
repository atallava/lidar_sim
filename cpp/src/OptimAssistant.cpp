#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <boost/filesystem.hpp>

#include <lidar_sim/OptimAssistant.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
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
    m_datestr_format("%H%d%m%y"),

    m_sim_version_triangle_src("250417"),
    m_num_nbrs_for_hit_prob(1),

    m_sim_type(-1),

    m_num_nbrs_for_blocks_sim(1),
    m_max_pts_for_blocks_sim(1e4),

    m_section_packet_step(1),
    m_slice_hit_to_blocks_threshold(0.5),
    m_slice_resn_along_ray(5),
    m_slice_miss_to_blocks_threshold(2),

    m_obj_calc_count(1)
{
    m_rel_path_poses_log = genPathPosesLog();
    m_imu_pose_server = PoseServer(m_rel_path_poses_log);
}

void OptimAssistant::init()
{
    if (m_verbose)
	std::cout << "OptimAssistant: init." << std::endl;

    // create directories for this instance
    m_instance_idx = getDateString(m_datestr_format);
    mkdirsForOptimInstance(m_instance_idx);
    // note: currently not optimizing triangle models
    // copy triangle models into relevant folders
    copyTriangleModels();

    // todo: assumes that some variables have been assigned. who checks that?

    std::string m_rel_path_section_for_model = 
	genPathSection(m_section_id_for_model);
    m_section_for_model = SectionLoader (m_rel_path_section_for_model);

    std::string m_rel_path_section_for_sim = 
	genPathSection(m_section_id_for_sim);
    m_section_for_sim = SectionLoader (m_rel_path_section_for_sim);

    if (m_sim_type == 0) {
	fillSectionPtIdsForBlocksSim();
    }
    else if (m_sim_type == 1) {
	createSimDetailTemplate();
    }
    else {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    m_initialized = true;
}

double OptimAssistant::calcObj(std::vector<double> x)
{
    if (!m_initialized)
    {
    	std::stringstream ss_err_msg;
    	ss_err_msg << "OptimAssistant has not been initialized.";
    	throw std::logic_error(ss_err_msg.str().c_str());
    }

    // create the ellipsoid block models
    if (m_verbose)
    	std::cout << "OptimAssistant: creating models." << std::endl;

    for (auto block_id : m_non_ground_block_ids)
    	buildModelsNonGroundBlock(block_id, x);

    // simulate
    if (m_verbose)
    	std::cout << "OptimAssistant: simulating." << std::endl;
    
    if (m_sim_type == 0)
	blocksSim();
    else if (m_sim_type == 1)
	sliceSim();
    else {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    // calculate error
    if (m_verbose)
    	std::cout << "OptimAssistant: calculating error." << std::endl;

    double obj = calcSimError();

    m_obj_calc_count++;
    
    return obj;
}

void OptimAssistant::buildModelsNonGroundBlock(const int block_id, const std::vector<double> x)
{
    std::string rel_path_pts = genPathNonGroundBlockPts(m_section_id_for_model, block_id);
    std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts, m_verbose);
    
    EllipsoidModeler modeler;
    modeler.setVerbosity(m_verbose);

    // parameters fed here
    modeler.m_n_clusters_per_pt = x[0];
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
    std::string rel_path_ellipsoids = genRelPathEllipsoids(m_section_id_for_model, block_id, m_obj_calc_count);
    modeler.writeEllipsoidsToFile(rel_path_ellipsoids);
}

void OptimAssistant::sliceSim()
{
    // ellipsoid model paths
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    for (auto block_id : m_non_ground_block_ids)
	rel_path_ellipsoid_model_blocks.push_back(
	    genRelPathEllipsoids(m_section_id_for_model, block_id, m_obj_calc_count));

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

    // todo: this is quite ugly. i only have blocks info for all blocks. not giving blocks info makes 
    // sim use all blocks
    // std::string rel_path_imu_posn_nodes = genPathImuPosnNodes(m_section_id_for_model);
    // std::string rel_path_block_node_ids_ground = genPathBlockNodeIdsGround(m_section_id_for_model);
    // std::string rel_path_block_node_ids_non_ground = genPathBlockNodeIdsNonGround(m_section_id_for_model);

    // sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);
    
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> sim_hit_flag;
    std::vector<std::vector<double> > real_pts;
    SimDetail sim_detail;
    sim_detail.setVerbosity(m_verbose);
    
    size_t n_origins = m_sim_detail_template.m_ray_origins.size();

    for (size_t i = 0; i < n_origins; ++i) {
	// get relevant information from sim detail template
	std::vector<double> ray_origin = m_sim_detail_template.m_ray_origins[i];
	std::vector<double> this_ray_pitches = m_sim_detail_template.m_ray_pitches[i];
	std::vector<double> this_ray_yaws = m_sim_detail_template.m_ray_yaws[i];
	std::vector<std::vector<double> > this_real_pts_all = m_sim_detail_template.m_real_pts_all[i];
	std::vector<int> this_real_hit_flag = m_sim_detail_template.m_real_hit_flags[i];

	std::vector<std::vector<double> > this_real_pts = logicalSubsetArray(this_real_pts_all, this_real_hit_flag);
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
    std::string rel_path_real_pts = genRelPathRealPts(m_section_id_for_sim, m_obj_calc_count);
    writePtsToXYZFile(real_pts, rel_path_real_pts, m_verbose);

    // write sim pts
    std::string rel_path_sim_pts = genRelPathSimPts(m_section_id_for_sim, m_obj_calc_count);
    writePtsToXYZFile(sim_pts, rel_path_sim_pts, m_verbose);

    // write sim detail
    std::string rel_path_sim_detail = genRelPathSimDetail(m_section_id_for_sim, m_obj_calc_count); 
    sim_detail.save(rel_path_sim_detail);  
}

void OptimAssistant::blocksSim()
{
    // ellipsoid model paths
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    for (auto block_id : m_non_ground_block_ids)
	rel_path_ellipsoid_model_blocks.push_back(
	    genRelPathEllipsoids(m_section_id_for_model, block_id, m_obj_calc_count));

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

    // todo: this is quite ugly. i only have blocks info for all blocks. not giving blocks info makes 
    // sim use all blocks
    // std::string rel_path_imu_posn_nodes = genPathImuPosnNodes(m_section_id_for_model);
    // std::string rel_path_block_node_ids_ground = genPathBlockNodeIdsGround(m_section_id_for_model);
    // std::string rel_path_block_node_ids_non_ground = genPathBlockNodeIdsNonGround(m_section_id_for_model);

    // sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);

    // sim
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> sim_hit_flag;
    std::vector<std::vector<double> > real_pts;
    SimDetail sim_detail;
    sim_detail.setVerbosity(m_verbose);
    for(auto idx : m_section_pt_ids_for_blocks_sim)
    {
    	double t = m_section_for_sim.m_pt_timestamps[idx];

    	// pose, ray origin, ray dirn
    	std::vector<double> imu_pose = m_imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);
	std::vector<double> this_real_pt = m_section_for_sim.m_pts[idx];
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
    std::string rel_path_real_pts = genRelPathRealPts(m_section_id_for_sim, m_obj_calc_count);
    writePtsToXYZFile(real_pts, rel_path_real_pts, m_verbose);

    // write sim pts
    std::string rel_path_sim_pts = genRelPathSimPts(m_section_id_for_sim, m_obj_calc_count);
    writePtsToXYZFile(sim_pts, rel_path_sim_pts, m_verbose);

    // write sim detail
    std::string rel_path_sim_detail = genRelPathSimDetail(m_section_id_for_sim, m_obj_calc_count); 
    sim_detail.save(rel_path_sim_detail);
}

double OptimAssistant::calcSimError()
{
    std::string rel_path_real_pts, rel_path_sim_pts, rel_path_sim_detail;
    double error;
    if (m_sim_type == 0) {
	// for blocks sim
	rel_path_real_pts = genRelPathRealPts(m_section_id_for_sim, m_obj_calc_count);
	rel_path_sim_pts = genRelPathSimPts(m_section_id_for_sim, m_obj_calc_count);
	std::vector<std::vector<double> > real_pts = loadPtsFromXYZFile(rel_path_real_pts, m_verbose);
	std::vector<std::vector<double> > sim_pts = loadPtsFromXYZFile(rel_path_sim_pts, m_verbose);
	error = m_error_metric.calcSymmetricPcdError(real_pts, sim_pts);
    }
    else if (m_sim_type == 1) {
	// for slice sim
	rel_path_sim_detail = genRelPathSimDetail(m_section_id_for_sim, m_obj_calc_count);
	SimDetail sim_detail(rel_path_sim_detail);
	double mean_packets_error, precision, recall;
	std::tie(mean_packets_error, precision, recall) = m_error_metric.calcRangeError(sim_detail);
	double f1_score = calcF1Score(precision, recall);

	// todo: check error
	// error = mean_packets_error;
	error = f1_score;
	// error = mean_packets_error + (1 - f1_score);
    }
    else {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    return error;
}

void OptimAssistant::fillSectionPtIdsForBlocksSim()
{
    // todo: assumes that some variables have been assigned. who checks that?

    bool condn = (m_sim_type == 0);
    if (!condn) {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    // ignoring ground block pts
    std::vector<int> non_ground_blocks_for_sim = m_non_ground_block_ids;
    for (auto block_id : non_ground_blocks_for_sim)
    {
	std::string rel_path_pts = genPathNonGroundBlockPts(m_section_id_for_sim, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts, m_verbose);

	std::vector<std::vector<int> > section_nbr_pt_ids; 

	std::tie(section_nbr_pt_ids, std::ignore) = 
	    nearestNeighbors(m_section_for_sim.m_pts, block_pts, m_num_nbrs_for_blocks_sim);

	for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	    for(size_t j = 0; j < (size_t)m_num_nbrs_for_blocks_sim; ++j)
	    {
		int idx = section_nbr_pt_ids[i][j];
		m_section_pt_ids_for_blocks_sim.push_back(idx);
	    }
    }

    // subsample pt ids for blocks sim
    std::vector<int> section_pt_ids_subsampled;
    int step = std::floor(m_section_pt_ids_for_blocks_sim.size()/m_max_pts_for_blocks_sim);
    if (step < 1)
	step = 1;
    for (size_t i = 0; i < m_section_pt_ids_for_blocks_sim.size(); i += step)
	section_pt_ids_subsampled.push_back(
	    m_section_pt_ids_for_blocks_sim[i]);
    m_section_pt_ids_for_blocks_sim = section_pt_ids_subsampled;
}

void OptimAssistant::createSimDetailTemplate()
{
    // todo: assumes that some variables have been assigned. who checks that?

    bool condn = (m_sim_type == 1);
    if (!condn) {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    std::vector<std::vector<double> > block_pts_all;
    // ground block pts
    for (auto block_id : m_ground_block_ids)
    {
	std::string path_pts = genPathGroundBlockPts(m_section_id_for_model, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(path_pts);
	block_pts_all.insert(block_pts_all.begin(), 
			     block_pts.begin(), block_pts.end());
    }

    // non ground block pts
    for (auto block_id : m_non_ground_block_ids)
    {
	std::string rel_path_pts = genPathNonGroundBlockPts(m_section_id_for_model, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);
	block_pts_all.insert(block_pts_all.begin(), 
			     block_pts.begin(), block_pts.end());
    }
    FlannDatasetWrapper flann_dataset_wrapper(block_pts_all);

    RayDirnServer ray_dirn_server;
    for (size_t i = m_section_packet_start; i < (size_t)m_section_packet_end; 
	 i += m_section_packet_step) {
	double t = m_section_for_sim.m_packet_timestamps[i];
	
	// pose, ray origin
    	std::vector<double> imu_pose = m_imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, m_laser_calib_params);

	// packet pts
    	std::vector<std::vector<double> > this_real_pts = m_section_for_sim.getPtsAtTime(t);

	// ray dirns
	std::vector<double> this_ray_pitches;
	std::vector<double> this_ray_yaws;
	std::vector<std::vector<double> > this_real_pts_all;
	std::vector<int> this_real_hit_flag;
	std::tie(this_ray_pitches, this_ray_yaws, this_real_pts_all, this_real_hit_flag)
	    = ray_dirn_server.fitDetailToPts(ray_origin, this_real_pts);
	std::vector<std::vector<double> > ray_dirns = calcRayDirnsFromSph(this_ray_pitches, this_ray_yaws);

	size_t n_rays = ray_dirns.size();
	std::vector<int> flag_for_template(n_rays, 0);
	
	// store hits if close to blocks
	std::vector<std::vector<double> > nn_dists_to_blocks_pts;
	std::tie(std::ignore, nn_dists_to_blocks_pts) = flann_dataset_wrapper.knnSearch(this_real_pts_all, 1);
	for (size_t j = 0; j < n_rays; ++j)
	{
	    bool condn1 = this_real_hit_flag[j];
	    bool condn2 = (nn_dists_to_blocks_pts[j][0] < m_slice_hit_to_blocks_threshold);
	    if (condn1 && condn2) 
		flag_for_template[j] = 1;
	}

	// store misses if close to blocks
	std::vector<double> ray_dists_to_blocks_pts;
	std::tie(std::ignore, ray_dists_to_blocks_pts) = 
	    flann_dataset_wrapper.approxNearestToRays(ray_origin, ray_dirns, 
						      m_laser_calib_params.intrinsics.max_range, m_slice_resn_along_ray);
	for (size_t j = 0; j < n_rays; ++j)
	{
	    bool condn1 = (!this_real_hit_flag[j]);
	    bool condn2 = (ray_dists_to_blocks_pts[j] < m_slice_miss_to_blocks_threshold);
	    if (condn1 && condn2) 
		flag_for_template[j] = 1;
	}

	// if all zeros, skip packet
	bool condn = anyNonzeros(flag_for_template);
	if (!condn)
	    continue;

	// add to sim detail template
	std::vector<double> pitches_for_template = logicalSubsetArray(this_ray_pitches, flag_for_template);
	std::vector<double> yaws_for_template = logicalSubsetArray(this_ray_yaws, flag_for_template);
	std::vector<std::vector<double> > real_pts_all_for_template = logicalSubsetArray(this_real_pts_all, flag_for_template);
	std::vector<int> real_hit_flag_for_template = logicalSubsetArray(this_real_hit_flag, flag_for_template);
	m_sim_detail_template.m_ray_origins.push_back(ray_origin);
	m_sim_detail_template.m_ray_pitches.push_back(pitches_for_template);
	m_sim_detail_template.m_ray_yaws.push_back(yaws_for_template);
	m_sim_detail_template.m_real_pts_all.push_back(real_pts_all_for_template);
	m_sim_detail_template.m_real_hit_flags.push_back(real_hit_flag_for_template);
    }
}


void OptimAssistant::mkdirsForOptimInstance(const std::string instance_idx)
{
    // instance dir
    std::string path_optim_instance_dir = genPathOptimInstanceDir(instance_idx);
    if (boost::filesystem::exists(path_optim_instance_dir)) {
	if (m_verbose)
	    std::cout << path_optim_instance_dir << " exists" << std::endl;
    }
    else {
	if (m_verbose)
	    std::cout << "creating " << path_optim_instance_dir << std::endl;
	boost::filesystem::create_directory(path_optim_instance_dir);
    }

    // models subdir
    std::ostringstream ss;
    ss << path_optim_instance_dir << "/models";
    std::string path_optim_models_dir = ss.str();
    if (boost::filesystem::exists(path_optim_models_dir)) {
	if (m_verbose)
	    std::cout << path_optim_models_dir << " exists" << std::endl;
    }
    else {
	if (m_verbose)
	    std::cout << "creating " << path_optim_models_dir << std::endl;
	boost::filesystem::create_directory(path_optim_models_dir);
    }

    // sim subdir
    ss.clear();
    ss.str("");
    ss << path_optim_instance_dir << "/sim";
    std::string path_optim_sim_dir = ss.str();
    if (boost::filesystem::exists(path_optim_sim_dir)) {
	if (m_verbose)
	    std::cout << path_optim_sim_dir << " exists" << std::endl;
    }
    else {
	if (m_verbose)
	    std::cout << "creating " << path_optim_sim_dir << std::endl;
	boost::filesystem::create_directory(path_optim_sim_dir);
    }
}


void OptimAssistant::copyTriangleModels()
{
    if (m_verbose)
	std::cout << "OptimAssistant: copyTriangleModels." << std::endl;

    for (size_t i = 0; i < m_ground_block_ids.size(); ++i)
    {
	int block_id = m_ground_block_ids[i];
	std::string rel_path_triangles_source = genRelPathTrianglesSrc(m_section_id_for_model, block_id);
	std::string rel_path_triangles_dest = genRelPathTriangles(m_section_id_for_model, block_id);
	if (boost::filesystem::exists(rel_path_triangles_dest)) {
	    if (m_verbose)
		std::cout << rel_path_triangles_dest << " exists." << std::endl;
	}
	else {
	    if (m_verbose)
		std::cout << "copying from " << rel_path_triangles_source << " to " 
			  << rel_path_triangles_dest << std::endl;
	    boost::filesystem::copy(rel_path_triangles_source, rel_path_triangles_dest);
	}
    }
}

void OptimAssistant::saveOptimConfig()
{
    std::string rel_path_config = genRelPathOptimConfig();
    std::string config_str = getConfigAsStr();
    writeStringToFile(config_str, rel_path_config, m_verbose);
}

std::string OptimAssistant::getConfigAsStr()
{
    std::ostringstream ss;
    ss << "OptimAssistant:" << std::endl;
    ss << "m_verbose: " << m_verbose << std::endl;
    ss << "m_instance_idx: " << m_instance_idx << std::endl;

    ss << "m_section_id_for_model: " << m_section_id_for_model << std::endl;
    ss << "m_rel_path_section_for_model: " << m_rel_path_section_for_model << std::endl;
    ss << "m_sim_version_triangle_src: " << m_sim_version_triangle_src << std::endl;
    ss << "m_non_ground_block_ids: " << std::endl;
    ss << getVecAsStr(m_non_ground_block_ids) << std::endl;
    ss << "m_ground_block_ids: " << std::endl;
    ss << getVecAsStr(m_ground_block_ids) << std::endl;
    ss << "m_num_nbrs_for_hit_prob: " << m_num_nbrs_for_hit_prob << std::endl;

    ss << "m_sim_type: " << m_sim_type << std::endl;
    ss << "m_section_id_for_sim: " << m_section_id_for_sim << std::endl;
    ss << "m_rel_path_section_for_sim: " << m_rel_path_section_for_sim << std::endl;

    if (m_sim_type == 0) {
	// ss << "m_section_pt_ids_for_blocks_sim: " << std::endl;
	// ss << getVecAsStr(m_section_pt_ids_for_blocks_sim) << std::endl;
	ss << "m_num_nbrs_for_blocks_sim: " << m_num_nbrs_for_blocks_sim << std::endl;
	ss << "m_max_pts_for_blocks_sim: " << m_max_pts_for_blocks_sim << std::endl;
    }
    else if (m_sim_type == 1) {
	ss << "m_section_packet_start: " << m_section_packet_start << std::endl;
	ss << "m_section_packet_end: " << m_section_packet_end << std::endl;
	ss << "m_section_packet_step: " << m_section_packet_step << std::endl;
	ss << "m_slice_hit_to_blocks_threshold: " << m_slice_hit_to_blocks_threshold << std::endl;
	ss << "m_slice_resn_along_ray: " << m_slice_resn_along_ray << std::endl;
	ss << "m_slice_miss_to_blocks_threshold: " << m_slice_miss_to_blocks_threshold << std::endl;
    }
    else {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }
    ss << "m_obj_calc_count: " << m_obj_calc_count;

    return ss.str();
}

std::string OptimAssistant::genPathOptimInstanceDir(const std::string instance_idx)
{
    std::ostringstream ss;
    ss << "/usr0/home/atallav1/lidar_sim/cpp"
       << "/data/sim_optim/instance_" << instance_idx;

    return ss.str();
}

std::string OptimAssistant::genRelPathEllipsoids(const int section_id, const int block_id, const int obj_calc_count)
{
    // todo: should the instance idx be passed as argument? or should the member
    // be used?  each object should ideally only access its own instance, so
    // member? but then that logic should be applied to all the variables passed
    // in? such as should use m_section_id_for_model only    

    std::ostringstream ss;
    ss << "data/sim_optim/instance_" << m_instance_idx
       << "/models" 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id 
       << "_non_ground_ellipsoids_" << obj_calc_count << ".txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sim_optim/instance_" << m_instance_idx
       << "/models"
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathTrianglesSrc(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/version_" << m_sim_version_triangle_src
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathRealPts(const int section_id, const int obj_calc_count)
{
    std::ostringstream ss;
    ss << "data/sim_optim/instance_" << m_instance_idx
       << "/sim"
       << "/section_" << std::setw(2) << std::setfill('0') << section_id;
    
    if (m_sim_type == 0) {
	ss << "_blocks";
    }
    else if (m_sim_type == 1) {
	ss << "_slice";
    }
    else {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    ss << "_real_pts_" << obj_calc_count << ".xyz";

    return ss.str();
}

std::string OptimAssistant::genRelPathSimPts(const int section_id, const int obj_calc_count)
{
    std::ostringstream ss;
    ss << "data/sim_optim/instance_" << m_instance_idx
       << "/sim"
       << "/section_" << std::setw(2) << std::setfill('0') << section_id;

    if (m_sim_type == 0) {
	ss << "_blocks";
    }
    else if (m_sim_type == 1) {
	ss << "_slice";
    }
    else {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    ss << "_sim_pts_" << obj_calc_count << ".xyz";

    return ss.str();
}

std::string OptimAssistant::genRelPathSimDetail(const int section_id, const int obj_calc_count)
{
    std::ostringstream ss;
    ss << "data/sim_optim/instance_" << m_instance_idx
       << "/sim"
       << "/section_" << std::setw(2) << std::setfill('0') << section_id;

    if (m_sim_type == 0) {
	ss << "_blocks";
    }
    else if (m_sim_type == 1) {
	ss << "_slice";
    }
    else {
	std::stringstream ss_err_msg;
	ss_err_msg << "m_sim_type has invalid value " << m_sim_type;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    ss << "_sim_detail_" << obj_calc_count << ".txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathOptimConfig()
{
    std::ostringstream ss;
    ss << "data/sim_optim/instance_" << m_instance_idx
       << "/optim_config.txt";

    return ss.str();
}

std::string OptimAssistant::genRelPathOptimProgress()
{
    std::ostringstream ss;
    ss << "data/sim_optim/instance_" << m_instance_idx
       << "/optim_progress.txt";

    return ss.str();
}


