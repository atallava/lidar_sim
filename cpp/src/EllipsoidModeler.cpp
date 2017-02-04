#include <algorithm>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// alglib stuff
#include "stdafx.h"
#include "dataanalysis.h"

#include "interpolation.h"

#include <lidar_sim/EllipsoidModeler.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/ClusteringUtils.h>

using namespace lidar_sim;

EllipsoidModeler::EllipsoidModeler() :
    m_debug_flag(0),
    m_min_pts_per_cluster(9),
    m_default_hit_prob(1),

    m_hit_count_prior(1),
    m_miss_count_prior(0)
{    
    m_n_clusters_per_pt = 250/(double)12016; // hack based on rim stretch test
}

void EllipsoidModeler::createEllipsoidModels(const std::string rel_path_pts)
{
    if (m_debug_flag)
	std::cout << "EllipsoidModeler: creating ellipsoid models..." << std::endl;

    loadPts(rel_path_pts);
    clusterPts();
    filterClusters();
    fillEllipsoidModels();
}

void EllipsoidModeler::loadPts(const std::string rel_path_pts)
{
    m_pts = loadPtsFromXYZFile(rel_path_pts);
}

void EllipsoidModeler::clusterPts()
{
    if (m_debug_flag)
	std::cout << "EllipsoidModeler: clustering..." << std::endl;

    alglib::real_2d_array pts_alglib = convertStlPtsToAlglibPts(m_pts);

    alglib::clusterizerstate clusterizer_state;
    alglib::ahcreport ahc_report;
    alglib::integer_1d_array cz;

    alglib::clusterizercreate(clusterizer_state);
    // last argument denotes euclidean distance
    alglib::clusterizersetpoints(clusterizer_state, pts_alglib, 2);
    alglib::clusterizerrunahc(clusterizer_state, ahc_report);

    // get clusters
    int m_n_clusters = calcNClusters();
    alglib::clusterizergetkclusters(ahc_report, m_n_clusters, m_pt_cluster_ids, cz);
}

void EllipsoidModeler::filterClusters()
{
    if (m_debug_flag)
	std::cout << "EllipsoidModeler: filtering clusters..." << std::endl;

    // retain those with min pts
    std::vector<int> n_pts_per_cluster = getNumPtsPerCluster(m_pt_cluster_ids, m_n_clusters);
    for(size_t i = 0; i < n_pts_per_cluster.size(); ++i)
	if (n_pts_per_cluster[i] >= m_min_pts_per_cluster)
	    m_selected_cluster_ids.push_back(i);

    if (m_debug_flag)
	std::cout << "n selected clusters: " << m_selected_cluster_ids.size() << std::endl;
}

void EllipsoidModeler::fillEllipsoidModels()
{
    if (m_debug_flag)
	std::cout << "EllipsoidModeler: filling ellipsoid models..." << std::endl;

    for(size_t i = 0; i < m_selected_cluster_ids.size(); ++i)
    {
	int this_cluster_id = m_selected_cluster_ids[i];

	// get pts corresponding to this cluster
	std::vector<std::vector<double> > this_cluster_pts;
	for(size_t j = 0; j < m_pts.size(); ++j)
	    if (m_pt_cluster_ids[j] == this_cluster_id)
		this_cluster_pts.push_back(m_pts[j]);
	
	m_ellipsoid_models.push_back(createEllipsoidModel(this_cluster_pts));
    }
}

int EllipsoidModeler::calcNClusters()
{
    return (int)m_pts.size()*m_n_clusters_per_pt;
}

EllipsoidModel EllipsoidModeler::createEllipsoidModel(const Pts &pts)
{
    EllipsoidModel model;
    model.mu = calcPtsMean(pts);
    model.cov_mat = calcPtsCovMat(pts);
    model.hit_prob = m_default_hit_prob;

    return model;
}

void EllipsoidModeler::writeEllipsoidsToFile(std::string rel_path_output)
{
    std::ofstream file(rel_path_output);
    std::cout << "EllipsoidModeler: writing ellipsoid models to: " << rel_path_output << std::endl;
	
    for(size_t i = 0; i < m_ellipsoid_models.size(); ++i)
    {
	// mu
	std::vector<double> mu = m_ellipsoid_models[i].mu;
	std::ostringstream line;
	for(size_t j = 0; j < 3; ++j)
	    line << mu[j] << " ";

	// cov mat
	Eigen::MatrixXd cov_mat = m_ellipsoid_models[i].cov_mat;
	for(size_t j = 0; j < 3; ++j)
	    for(size_t k = 0; k < 3; ++k)
		line << cov_mat(k,j) << " ";
	    
	// hit_prob
	double hit_prob = m_ellipsoid_models[i].hit_prob;
	line << hit_prob << std::endl;
	    
	file << line.str();
    }

    file.close();
}

void EllipsoidModeler::calcHitProb(std::string rel_path_section, PoseServer imu_pose_server)
{
    if (m_debug_flag)
	std::cout << "EllipsoidModeler: calculating hot probs..." << std::endl;

    SectionLoader section(rel_path_section);

    // section ids to process
    std::vector<int> section_pt_ids_to_process;
    std::tie(section_pt_ids_to_process, std::ignore) = nearestNeighbors(section.m_pts, m_pts);

    // sim object
    // odd that modeler needs a sim
    const LaserCalibParams laser_calib_params;
    EllipsoidModelSim sim;
    sim.setEllipsoidModels(m_ellipsoid_models);
    sim.setLaserCalibParams(laser_calib_params);

    size_t n_ellipsoids = m_ellipsoid_models.size();
    std::vector<int> ellipsoid_hit_count_prior(n_ellipsoids, m_hit_count_prior);
    std::vector<int> ellipsoid_miss_count_prior(n_ellipsoids, m_miss_count_prior);

    std::vector<int> ellipsoid_hit_count = ellipsoid_hit_count_prior;
    std::vector<int> ellipsoid_miss_count = ellipsoid_miss_count_prior;

    for(size_t i = 0; i < section_pt_ids_to_process.size(); ++i)
    {
    	int id = section_pt_ids_to_process[i];
	
    	double t = section.m_pt_timestamps[id];
    	std::vector<double> this_pt = section.m_pts[id];
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, laser_calib_params);
    	std::vector<double> ray_dirn;
    	double meas_dist;
    	std::tie(ray_dirn, meas_dist) = calcRayDirn(ray_origin, this_pt);
	
    	std::vector<int> intersection_flag;
	std::vector<double> dist_along_ray;
	std::tie(intersection_flag, dist_along_ray) = sim.calcEllipsoidIntersections(
    	    ray_origin, ray_dirn);

    	if (!anyNonzeros(intersection_flag))
    	    continue; 	// no hits

    	std::vector<int> sorted_intersecting_ids;
    	std::vector<double> sorted_dist_along_ray;
    	std::tie(sorted_intersecting_ids, sorted_dist_along_ray) =
    	    sortIntersectionFlag(intersection_flag, dist_along_ray);
    	std::vector<double> maha_dists_to_ellipsoids = 
    	    sim.calcMahaDistPtToEllipsoids(sorted_intersecting_ids, this_pt);
	
    	int ellipsoid_hit_id;
    	std::vector<int> ellipsoid_miss_ids;
    	std::tie(ellipsoid_hit_id, ellipsoid_miss_ids) = 
    	    sim.assignEllipsoidHitCredits(maha_dists_to_ellipsoids, sorted_intersecting_ids);

	// assign credits
	if (ellipsoid_hit_id != -1)
	    ellipsoid_hit_count[ellipsoid_hit_id] += 1;

	if (!ellipsoid_miss_ids.empty())
	    for(size_t j = 0; j < ellipsoid_miss_ids.size(); ++j)
		ellipsoid_miss_count[ellipsoid_miss_ids[j]] += 1;

	// debug
	// std::cout << "section pt id: " << id << std::endl;
	// std::cout << "t: " << t << std::endl;
	// std::cout << "this pt: " << std::endl;
	// dispVec(this_pt);
	// std::cout << "imu_pose " << std::endl;
	// dispVec(imu_pose);
	// std::cout << "ray origin " << std::endl;
	// dispVec(ray_origin);
	// std::cout << "ray dirn " << std::endl;
	// dispVec(ray_dirn);
	// std::cout << "meas dist " << meas_dist << std::endl;
	// std::cout << "sorted intersecting ids " << std::endl;
	// dispVec(sorted_intersecting_ids);
	// std::cout << "sorted dist along ray " << std::endl;
	// dispVec(sorted_dist_along_ray);
	// std::cout << "maha dists to ellipsoids " << std::endl;
	// dispVec(maha_dists_to_ellipsoids);
	// std::cout << "ellipsoid hit id: " << ellipsoid_hit_id << std::endl;
	// std::cout << "ellipsoid miss ids: "  << std::endl;
	// dispVec(ellipsoid_miss_ids);
    }

	// calc hit prob and add to models
	std::vector<double> hit_prob_vec(m_ellipsoid_models.size(), 1);
	for(size_t i = 0; i < hit_prob_vec.size(); ++i)
	{
	    hit_prob_vec[i] = (double)(ellipsoid_hit_count[i]/(double)(ellipsoid_hit_count[i] + ellipsoid_miss_count[i]));
	    m_ellipsoid_models[i].hit_prob = hit_prob_vec[i];
	}
}

void EllipsoidModeler::setDebugFlag(int flag)
{
    m_debug_flag = flag;
}
