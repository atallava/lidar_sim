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
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/ClusteringUtils.h>

using namespace lidar_sim;

EllipsoidModeler::EllipsoidModeler() :
    m_debug_flag(0),
    m_min_pts_per_cluster(9)
{    
    m_n_clusters_per_pt = 2300/(double)12016; // hack based on rim stretch test
}

void EllipsoidModeler::createEllipsoidModels(const std::string rel_path_pts)
{
    loadPts(rel_path_pts);
    size_t n_pts = m_pts.size();
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
    return m_pts.size()*m_n_clusters_per_pt;
}

EllipsoidModel EllipsoidModeler::createEllipsoidModel(const Pts &pts)
{
    EllipsoidModel model;
    model.mu = calcPtsMean(pts);
    model.cov_mat = calcPtsCovMat(pts);
    model.hit_prob = 1;

    return model;
}

void EllipsoidModeler::writeEllipsoidModelsToFile(std::string rel_path_output)
{
    std::ofstream file(rel_path_output);
    std::cout << "Writing ellipsoid models to: " << rel_path_output << std::endl;
	
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

