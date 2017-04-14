#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/PtsError.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // error metric
    PtsError metric;

    // hg sim
    std::string rel_path_hg_real_pts = "data/hg_real_pts.xyz";
    std::vector<std::vector<double> > hg_real_pts = loadPtsFromXYZFile(rel_path_hg_real_pts);
    
    std::string rel_path_hg_sim_pts = "data/hg_sim_pts.xyz";
    std::vector<std::vector<double> > hg_sim_pts = loadPtsFromXYZFile(rel_path_hg_sim_pts);

    double hg_loss_mean, hg_loss_var;
    std::tie(hg_loss_mean, hg_loss_var) = metric.calcError(hg_real_pts, hg_sim_pts);
    // std::tie(hg_loss_mean, hg_loss_var) = metric.calcError(hg_sim_pts, hg_real_pts);
    
    std::cout << "hg sim: " << std::endl;
    std::cout << "loss mean: " << hg_loss_mean << " loss var: " << hg_loss_var << std::endl;

    // nbr sim
    std::string rel_path_nbr_real_pts = "data/nbr_real_pts.xyz";
    std::vector<std::vector<double> > nbr_real_pts = loadPtsFromXYZFile(rel_path_nbr_real_pts);
    
    std::string rel_path_nbr_sim_pts = "data/nbr_sim_pts.xyz";
    std::vector<std::vector<double> > nbr_sim_pts = loadPtsFromXYZFile(rel_path_nbr_sim_pts);

    double nbr_loss_mean, nbr_loss_var;
    std::tie(nbr_loss_mean, nbr_loss_var) = metric.calcError(nbr_real_pts, nbr_sim_pts);
    // std::tie(nbr_loss_mean, nbr_loss_var) = metric.calcError(nbr_sim_pts, nbr_real_pts);

    std::cout << "nbr sim: " << std::endl;
    std::cout << "loss mean: " << nbr_loss_mean << " loss var: " << nbr_loss_var << std::endl;

    // sanity check
    // that the real pts from both are the same
    double sanity_loss_mean, sanity_loss_var;
    std::tie(sanity_loss_mean, sanity_loss_var) = metric.calcError(hg_real_pts, nbr_real_pts);
    std::cout << "sanity check" << std::endl;
    std::cout << "loss mean: " << sanity_loss_mean << " loss var: " << sanity_loss_var << std::endl;

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
