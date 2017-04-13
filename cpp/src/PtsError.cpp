#include <numeric>

#include <lidar_sim/PtsError.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

PtsError::PtsError() 
{
}

std::tuple<double, double>
PtsError::calcError(const std::vector<std::vector<double> > &pts1, 
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
