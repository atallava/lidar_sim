#include <numeric>
#include <iostream>

#include <lidar_sim/PtsError.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

PtsError::PtsError() 
{
}

std::tuple<double, double>
PtsError::calcAsymmetricError(const std::vector<std::vector<double> > &pts1, 
			      const std::vector<std::vector<double> > &pts2)
{
    std::vector<double> error_vec(pts2.size(), 0);

    std::vector<std::vector<int> > nn_ids;
    std::vector<std::vector<double> > nn_dists;
    std::tie(nn_ids, nn_dists) = nearestNeighbors(pts1, pts2, 1);

    std::vector<double> pt_losses;
    for(size_t i = 0; i < nn_dists.size(); ++i)
	pt_losses.push_back(nn_dists[i][0]);
    
    return calcVecMeanVar(pt_losses);
}

double PtsError::calcSymmetricError(const std::vector<std::vector<double> > &pts1, 
				    const std::vector<std::vector<double> > &pts2)
{
    double error_12, error_21;
    std::tie(error_12, std::ignore) = calcAsymmetricError(pts1, pts2);
    std::tie(error_21, std::ignore) = calcAsymmetricError(pts2, pts1);
    double error = (error_12 + error_21)/2.0;

    return error;
}

void PtsError::calcRangeError(const SimDetail &sim_detail)
{
    size_t n_origins = sim_detail.m_ray_origins.size();
    std::vector<double> err_vec(n_origins, 0.0);
    std::vector<double> err_vec_only_hits(n_origins, 0.0);
    std::vector<int> n_real_pts_vec(n_origins, 0.0);
    std::vector<int> n_misses_vec(n_origins, 0.0);
    std::vector<double> fracn_misses_vec(n_origins, 0.0);

    for(size_t i = 0; i < n_origins; ++i)
    {
	std::vector<double> ray_origin = sim_detail.m_ray_origins[i];
	std::vector<std::vector<double> > real_pts = 
	    sim_detail.m_real_pts[i];
	std::vector<std::vector<double> > sim_pts = 
	    sim_detail.m_sim_pts[i];
	std::vector<int> hit_flag = sim_detail.m_hit_flags[i];

	size_t n_real_pts = real_pts.size();
	n_real_pts_vec[i] = n_real_pts;

	std::vector<double> real_ranges(n_real_pts, 0.0);
	std::vector<double> sim_ranges(n_real_pts, 0.0);

	std::vector<double> this_err_vec;
	std::vector<double> this_err_vec_only_hits;

	int n_misses = n_real_pts;
	for(size_t j = 0; j < n_real_pts; ++j)
	{
	    double real_range = euclideanDist(ray_origin, real_pts[j]);
	    double sim_range = euclideanDist(ray_origin, sim_pts[j]);

	    real_ranges[j] = real_range;
	    sim_ranges[j] = sim_range;

	    double err = std::pow(real_range-sim_range, 2.0);
	    this_err_vec.push_back(err);
	    if (hit_flag[j])
	    {
		this_err_vec_only_hits.push_back(err);
		n_misses--;
	    }
	}

	std::tie(err_vec[i], std::ignore) = calcVecMeanVar(this_err_vec);
	if (n_misses < n_real_pts)
	    std::tie(err_vec_only_hits[i], std::ignore) = calcVecMeanVar(this_err_vec_only_hits);

	n_misses_vec[i] = n_misses;
	fracn_misses_vec[i] = n_misses/n_real_pts;
    }
    
    double err_mean, err_var; 
    std::tie(err_mean, err_var) = calcVecMeanVar(err_vec);
    
    double err_only_hits_mean, err_only_hits_var; 
    std::tie(err_only_hits_mean, err_only_hits_var) = calcVecMeanVar(err_vec_only_hits);

    int overall_n_misses = std::accumulate(n_misses_vec.begin(), n_misses_vec.end(), 0.0);
    int overall_n_real_pts = std::accumulate(n_real_pts_vec.begin(), n_real_pts_vec.end(), 0.0);
    double overall_fracn_misses = overall_n_misses/overall_n_real_pts;
    
    std::cout << "err. mean: " << err_mean << " var: " << err_var << std::endl;
    std::cout << "err_only_hits. mean: " << err_only_hits_mean << " var: " << err_only_hits_var << std::endl;
    std::cout << "overall fracn misses: " << overall_fracn_misses << std::endl;;
}

