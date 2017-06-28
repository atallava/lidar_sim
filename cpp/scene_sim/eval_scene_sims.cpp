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
#include <lidar_sim/SimDetail.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // error metric
    PtsError metric;

    // hg sim
    std::string rel_path_hg_real_pts = "data/sections/section_04/hg_sim/slice_real_pts.xyz";
    std::vector<std::vector<double> > hg_real_pts = loadPtsFromXYZFile(rel_path_hg_real_pts);
    
    std::string rel_path_hg_sim_pts = "data/sections/section_04/hg_sim/slice_sim_pts.xyz";
    std::vector<std::vector<double> > hg_sim_pts = loadPtsFromXYZFile(rel_path_hg_sim_pts);

    double hg_loss_mean, hg_loss_var;
    // std::tie(hg_loss_mean, hg_loss_var) = metric.calcAsymmetricError(hg_real_pts, hg_sim_pts);
    std::tie(hg_loss_mean, hg_loss_var) = metric.calcAsymmetricError(hg_sim_pts, hg_real_pts);
    
    std::cout << "hg sim: " << std::endl;
    std::cout << "loss mean: " << hg_loss_mean << " loss var: " << hg_loss_var << std::endl;

    // mesh model sim
    std::string rel_path_mm_real_pts = "data/sections/section_04/mesh_model_sim/slice_real_pts.xyz";
    std::vector<std::vector<double> > mm_real_pts = loadPtsFromXYZFile(rel_path_mm_real_pts);
    
    std::string rel_path_mm_sim_pts = "data/sections/section_04/mesh_model_sim/slice_sim_pts.xyz";
    std::vector<std::vector<double> > mm_sim_pts = loadPtsFromXYZFile(rel_path_mm_sim_pts);

    double mm_loss_mean, mm_loss_var;
    // std::tie(mm_loss_mean, mm_loss_var) = metric.calcAsymmetricError(mm_real_pts, mm_sim_pts);
    std::tie(mm_loss_mean, mm_loss_var) = metric.calcAsymmetricError(mm_sim_pts, mm_real_pts);
 
    std::cout << "mm sim: " << std::endl;
    std::cout << "loss mean: " << mm_loss_mean << " loss var: " << mm_loss_var << std::endl;

    // sanity check
    // that the real pts from both are the same
    double sanity_loss_mean, sanity_loss_var;
    std::tie(sanity_loss_mean, sanity_loss_var) = metric.calcAsymmetricError(hg_real_pts, mm_real_pts);
    std::cout << "sanity check" << std::endl;
    std::cout << "loss mean: " << sanity_loss_mean << " loss var: " << sanity_loss_var << std::endl;

    // range error
    std::string rel_path_sim_detail_hg = "data/sections/section_04/hg_sim/slice_sim_detail.txt";
    SimDetail sim_detail_hg(rel_path_sim_detail_hg);
    std::string rel_path_sim_detail_mm = "data/sections/section_04/mesh_model_sim/slice_sim_detail.txt";
    SimDetail sim_detail_mm(rel_path_sim_detail_mm);

    std::cout << "hg range error" << std::endl;
    metric.calcRangeError(sim_detail_hg);
    std::cout << "mm range error" << std::endl;
    metric.calcRangeError(sim_detail_mm);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
