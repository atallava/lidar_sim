#include <numeric>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <lidar_sim/GeometricSegmenter.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

GeometricSegmenter::GeometricSegmenter() :
    m_max_dist2_to_nbr(25),
    m_min_nbrs(15),
    m_default_spherical_variation(0.01),
    m_spherical_variation_threshold(0.09),
    m_debug_flag(0)
{
}

void GeometricSegmenter::setDebugFlag(int flag)
{
    m_debug_flag = flag;
}

std::vector<int> GeometricSegmenter::segmentPts(const std::vector<std::vector<double> > &pts)
{
    // 1 is ground, 0 is non-ground
    
    std::vector<int> segmentation(pts.size(), 0);

    std::vector<double> feature_values = calcFeaturesForPts(pts);

    for(size_t i = 0; i < pts.size(); ++i)
	if (feature_values[i] < m_spherical_variation_threshold)
	    segmentation[i] = 1;

    if (m_debug_flag)
	std::cout << "GeometricSegmenter: fracn ground pts: " 
		  << std::accumulate(std::begin(segmentation), std::end(segmentation), 0)/(double)segmentation.size() 
		  << std::endl;

    return segmentation;
}


std::vector<double> GeometricSegmenter::calcFeaturesForPts(const std::vector<std::vector<double> > &pts)
{
    std::vector<double> feature_values(pts.size(), 0);

    std::vector<std::vector<int> > nn_ids;
    std::vector<std::vector<double> > nn_dists;
    std::tie(nn_ids, nn_dists) = nearestNeighbors(pts, pts, m_min_nbrs);

    int n_pts_insufficient_nbrs = 0;
    for(size_t i = 0; i < pts.size(); ++i)
    {
	std::vector<std::vector<double> > this_pt_nbrs;
	for(size_t j = 0; j < nn_ids[0].size(); ++j)
	    if (nn_dists[i][j] < m_max_dist2_to_nbr)
		this_pt_nbrs.push_back(pts[nn_ids[i][j]]);

	if (this_pt_nbrs.size() < m_min_nbrs)
	{
	    n_pts_insufficient_nbrs++;
	    feature_values[i] = m_default_spherical_variation;
	}
	else
	    feature_values[i] = calcSphericalVariation(this_pt_nbrs);
    }

    if (m_debug_flag)
	std::cout << "GeometricSegmenter: fracn with insufficient nbrs: " << n_pts_insufficient_nbrs/(double)pts.size() 
		  << std::endl;

    return feature_values;
}

double GeometricSegmenter::calcSphericalVariation(const std::vector<std::vector<double> > &pts)
{
    Eigen::MatrixXd cov_mat = calcPtsCovMat(pts);
    Eigen::VectorXcd eivals = cov_mat.eigenvalues();
    std::vector<double> eigenvalues(3,0);
    for(size_t i = 0; i < 3; ++i)
	eigenvalues[i] = std::real(eivals(i));
    std::sort(eigenvalues.begin(), eigenvalues.end());

    return eigenvalues[0]/std::accumulate(eigenvalues.begin(), eigenvalues.end(), 0.0);
}
