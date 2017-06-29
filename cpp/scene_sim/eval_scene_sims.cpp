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

    // pcd error
    std::cout << "pcd error: " << std::endl;

    // hg sim
    std::string rel_path_hg_real_pts = "data/sections/section_04/hg_sim/slice_real_pts.xyz";
    std::vector<std::vector<double> > hg_real_pts = loadPtsFromXYZFile(rel_path_hg_real_pts);
    
    std::string rel_path_hg_sim_pts = "data/sections/section_04/hg_sim/slice_sim_pts.xyz";
    std::vector<std::vector<double> > hg_sim_pts = loadPtsFromXYZFile(rel_path_hg_sim_pts);

    std::cout << "hg sim: " << std::endl;
    metric.dispPcdError(hg_real_pts, hg_sim_pts);

    // mesh model sim
    std::string rel_path_mm_real_pts = "data/sections/section_04/mesh_model_sim/slice_real_pts.xyz";
    std::vector<std::vector<double> > mm_real_pts = loadPtsFromXYZFile(rel_path_mm_real_pts);
    
    std::string rel_path_mm_sim_pts = "data/sections/section_04/mesh_model_sim/slice_sim_pts.xyz";
    std::vector<std::vector<double> > mm_sim_pts = loadPtsFromXYZFile(rel_path_mm_sim_pts);

    std::cout << "mm sim: " << std::endl;
    metric.dispPcdError(mm_real_pts, mm_sim_pts);

    // sanity check
    std::cout << std::endl;
    std::cout << "sanity check. mm, hg real pts: " << std::endl;
    metric.dispPcdError(hg_real_pts, mm_real_pts);

    // range error
    std::cout << std::endl;
    std::cout << "range error: " << std::endl;

    // hg sim
    std::string rel_path_sim_detail_hg = "data/sections/section_04/hg_sim/slice_sim_detail.txt";
    SimDetail sim_detail_hg(rel_path_sim_detail_hg);

    std::cout << "hg sim:" << std::endl;
    metric.dispRangeError(sim_detail_hg);

    // mm sim
    std::string rel_path_sim_detail_mm = "data/sections/section_04/mesh_model_sim/slice_sim_detail.txt";
    SimDetail sim_detail_mm(rel_path_sim_detail_mm);

    std::cout << "mm sim:" << std::endl;
    metric.dispRangeError(sim_detail_mm);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
