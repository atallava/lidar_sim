#include <numeric>
#include <iostream>

#include <lidar_sim/PtsError.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

PtsError::PtsError() 
{
}

std::tuple<double, double>
PtsError::calcAsymmetricPcdError(const std::vector<std::vector<double> > &pts1, 
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

double PtsError::calcSymmetricPcdError(const std::vector<std::vector<double> > &pts1, 
				       const std::vector<std::vector<double> > &pts2)
{
    double error_12, error_21;
    std::tie(error_12, std::ignore) = calcAsymmetricPcdError(pts1, pts2);
    std::tie(error_21, std::ignore) = calcAsymmetricPcdError(pts2, pts1);
    double error = (error_12 + error_21)/2.0;

    return error;
}

void PtsError::dispPcdError(const std::vector<std::vector<double> > &pts1, 
			    const std::vector<std::vector<double> > &pts2)
{
    double err_mean_12, err_mean_21;
    double err_var_12, err_var_21;
    std::tie(err_mean_12, err_var_12) = calcAsymmetricPcdError(pts1, pts2);
    std::tie(err_mean_21, err_var_21) = calcAsymmetricPcdError(pts2, pts1);
    double err_mean_sym = (err_mean_12 + err_mean_21)/2.0;

    std::cout << "error 12. mean: " << err_mean_12 << " var: " << err_var_12 << std::endl;
    std::cout << "error 21. mean: " << err_mean_21 << " var: " << err_var_21 << std::endl;
    std::cout << "error mean sym: " << err_mean_sym << std::endl;
}

void PtsError::dispRangeError(const SimDetail &sim_detail)
{
    size_t n_packets = sim_detail.m_ray_origins.size();
    std::vector<double> packet_mean_errors(n_packets, 0);
    std::vector<int> packet_true_hits(n_packets, 0);
    std::vector<int> packet_false_misses(n_packets, 0);
    std::vector<int> packet_false_hits(n_packets, 0);
    std::vector<int> packet_true_misses(n_packets, 0);

    for(size_t i = 0; i < n_packets; ++i)
    {
	std::vector<std::vector<double> > this_packet_real_pts_all = sim_detail.m_real_pts_all[i];
	std::vector<int> this_packet_real_hit_flag = sim_detail.m_real_hit_flags[i];
	std::vector<std::vector<double> > this_packet_sim_pts_all = sim_detail.m_sim_pts_all[i];
	std::vector<int> this_packet_sim_hit_flag = sim_detail.m_sim_hit_flags[i];

	size_t n_rays = this_packet_real_pts_all.size();
	std::vector<double> this_packet_errors;
	int this_packet_true_hits = 0;
	int this_packet_false_misses = 0;
	int this_packet_false_hits = 0;
	int this_packet_true_misses = 0;
	for(size_t j = 0; j < n_rays; ++j)
	{
	    bool condn1 = (this_packet_real_hit_flag[j] == 1);
	    bool condn2 = (this_packet_sim_hit_flag[j] == 1);
	    
	    if (condn1 && !condn2)
		this_packet_false_misses++;
	    
	    if (condn1 && condn2)
	    {
		this_packet_true_hits++;
		double err = vectorNorm(
		    vectorDiff(this_packet_real_pts_all[j], this_packet_sim_pts_all[j]));
		this_packet_errors.push_back(err);
	    }

	    if (!condn1 && condn2)
		this_packet_false_hits++;

	    if (!condn1 && !condn2)
		this_packet_true_misses++;
	}
	
	double this_packet_mean_error = 0;
	if (this_packet_true_hits > 0)
	    this_packet_mean_error = std::accumulate(this_packet_errors.begin(), 
						     this_packet_errors.end(), 0.0)/this_packet_true_hits;
        
	packet_mean_errors[i] = this_packet_mean_error;
	packet_true_hits[i] = this_packet_true_hits;
	packet_false_misses[i] = this_packet_false_misses;
	packet_false_hits[i] = this_packet_false_hits;
	packet_true_misses[i] = this_packet_true_misses;
    }

    std::cout << "packet mean errors" << std::endl;
    dispVecMeanVar(packet_mean_errors);
    std::cout << "packet true hits" << std::endl;
    dispVecMeanVar(packet_true_hits);
    std::cout << "packet false misses" << std::endl;
    dispVecMeanVar(packet_false_misses);
    std::cout << "packet false hits" << std::endl;
    dispVecMeanVar(packet_false_hits);
    std::cout << "packet true misses" << std::endl;
    dispVecMeanVar(packet_true_misses);
}

