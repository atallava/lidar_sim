#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

int main(int argc, char**argv)
{
    std::vector<std::vector<double> > pts;
    for(size_t i = 0; i < 5; ++i)
    {
	std::vector<double> pt(1,i);
	pts.push_back(pt);
    }

    std::vector<std::vector<int> > nn_ids;
    std::vector<std::vector<double> > nn_dists;
    std::tie(nn_ids, nn_dists) = nearestNeighbors(pts, pts, 3);

    std::cout << "nn ids: " << std::endl;
    dispMat(nn_ids);
    std::cout << "nn dists: " << std::endl;
    dispMat(nn_dists);

    return(1);
}
