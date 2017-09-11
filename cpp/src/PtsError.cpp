#include <numeric>
#include <iostream>

#include <lidar_sim/PtsError.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/VizUtils.h>

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

std::tuple<double, double, double> PtsError::calcRangeError(const SimDetail &sim_detail, int disp)
{
    size_t n_packets = sim_detail.m_ray_origins.size();
    std::vector<int> packet_true_hits(n_packets, 0);
    std::vector<int> packet_false_misses(n_packets, 0);
    std::vector<int> packet_false_hits(n_packets, 0);
    std::vector<int> packet_true_misses(n_packets, 0);
    std::vector<std::vector<double> > packet_hit_errors;
    std::vector<std::vector<double> > packet_true_hits_real_ranges;

    for(size_t i = 0; i < n_packets; ++i)
    {
	std::vector<double> this_packet_ray_origin = sim_detail.m_ray_origins[i];
	std::vector<std::vector<double> > this_packet_real_pts_all = sim_detail.m_real_pts_all[i];
	std::vector<int> this_packet_real_hit_flag = sim_detail.m_real_hit_flags[i];
	std::vector<std::vector<double> > this_packet_sim_pts_all = sim_detail.m_sim_pts_all[i];
	std::vector<int> this_packet_sim_hit_flag = sim_detail.m_sim_hit_flags[i];

	size_t n_rays = this_packet_real_pts_all.size();
	int this_packet_true_hits = 0;
	int this_packet_false_misses = 0;
	int this_packet_false_hits = 0;
	int this_packet_true_misses = 0;
	std::vector<double> this_packet_hit_errors;
	std::vector<double> this_packet_true_hits_real_ranges;
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
		this_packet_hit_errors.push_back(err);

		double real_range = vectorNorm(
		    vectorDiff(this_packet_real_pts_all[j], this_packet_ray_origin));
		this_packet_true_hits_real_ranges.push_back(real_range);
	    }

	    if (!condn1 && condn2)
		this_packet_false_hits++;

	    if (!condn1 && !condn2)
		this_packet_true_misses++;
	}
	
	packet_true_hits[i] = this_packet_true_hits;
	packet_false_misses[i] = this_packet_false_misses;
	packet_false_hits[i] = this_packet_false_hits;
	packet_true_misses[i] = this_packet_true_misses;
	packet_hit_errors.push_back(this_packet_hit_errors);
	packet_true_hits_real_ranges.push_back(this_packet_true_hits_real_ranges);
    }

    double total_true_hits = std::accumulate(packet_true_hits.begin(), packet_true_hits.end(), 0.0);
    double total_false_hits = std::accumulate(packet_false_hits.begin(), packet_false_hits.end(), 0.0);
    double total_false_misses = std::accumulate(packet_false_misses.begin(), packet_false_misses.end(), 0.0);
    double precision = total_true_hits/(total_true_hits + total_false_hits);
    double recall = total_true_hits/(total_true_hits + total_false_misses);
    double f1_score = calcF1Score(precision, recall);

    // marginalize over packets
    std::vector<double> hit_errors_across_packets;
    for (size_t i = 0; i < packet_hit_errors.size(); ++i)
	for (size_t j = 0; j < packet_hit_errors[i].size(); ++j)
	    hit_errors_across_packets.push_back(packet_hit_errors[i][j]);

    std::vector<double> true_hits_real_ranges_across_packets;
    for (size_t i = 0; i < packet_true_hits_real_ranges.size(); ++i)
	for (size_t j = 0; j < packet_true_hits_real_ranges[i].size(); ++j)
	    true_hits_real_ranges_across_packets.push_back(
		packet_true_hits_real_ranges[i][j]);

    if (disp) {
	std::cout << "hit error across packets: " << std::endl;
	dispVecMeanVar(hit_errors_across_packets);
	std::cout << "real ranges corresponding to true hits, across packets: " << std::endl;
	dispVecMeanVar(true_hits_real_ranges_across_packets);
	std::cout << "total true hits: " << total_true_hits
		  << ", total false hits: " << total_false_hits << std::endl;
	std::cout << "total false misses: " << total_false_misses << std::endl;
	std::cout << "precision: " << precision << ", recall: " << recall 
		  << ", f1: " << f1_score << std::endl;
    } 

    double hit_errors_mean;
    std::tie(hit_errors_mean, std::ignore) = calcVecMeanVar(hit_errors_across_packets);
    return std::tie(hit_errors_mean, precision, recall);
}
