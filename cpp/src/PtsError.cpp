#include <lidar_sim/PtsError.h>

using namespace lidar_sim;

PtsError::PtsError() 
{
}

double PtsError::calcError(const std::vector<std::vector<double> > &pts1, 
			   const std::vector<std::vector<double> > &pts2)
{
    std::vector<double> error_vec(pts2.size(), 0);

    std::vector<std::vector<int> > nn_ids;
    std::vector<std::vector<double> > nn_dists;
    double mean = std::accumulate(nn_dists.begin(), nn_dists.end(), 0.0)/nn_dists.size();
    return mean;
}
