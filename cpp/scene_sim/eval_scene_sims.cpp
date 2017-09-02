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

    int section_id = 4;
    int tag = -1;
    std::string query_type = "slice";
    std::string hg_sim_version = "290717";
    std::string mm_sim_version = "290717";

    // error metric
    PtsError metric;

    // pcd error
    std::cout << "pcd error: " << std::endl << std::endl;

    // hg sim
    std::cout << "hg sim, version " << hg_sim_version << " : " << std::endl;
    std::string rel_path_hg_real_pts = genPathRealPtsRef(section_id, "hg", hg_sim_version, 
							 query_type, tag);
    std::vector<std::vector<double> > hg_real_pts = loadPtsFromXYZFile(rel_path_hg_real_pts);
    std::string rel_path_hg_sim_pts =  genPathSimPts(section_id, "hg", hg_sim_version, 
							 query_type, tag);
    std::vector<std::vector<double> > hg_sim_pts = loadPtsFromXYZFile(rel_path_hg_sim_pts);

    metric.dispPcdError(hg_real_pts, hg_sim_pts);

    dispHorizontalLine(25);

    // mm sim
    std::cout << "mm sim, version " << mm_sim_version << " : " << std::endl;
    std::string rel_path_mm_real_pts =  genPathRealPtsRef(section_id, "mm", mm_sim_version, 
							   query_type, tag);
    std::vector<std::vector<double> > mm_real_pts = loadPtsFromXYZFile(rel_path_mm_real_pts);
    std::string rel_path_mm_sim_pts =   genPathSimPts(section_id, "mm", mm_sim_version, 
						       query_type, tag);
    std::vector<std::vector<double> > mm_sim_pts = loadPtsFromXYZFile(rel_path_mm_sim_pts);

    metric.dispPcdError(mm_real_pts, mm_sim_pts);

    dispHorizontalLine(25);
    // sanity check
    std::cout << std::endl;
    std::cout << "sanity check. mm, hg real pts: " << std::endl;
    metric.dispPcdError(hg_real_pts, mm_real_pts);

    dispHorizontalLine();

    // range error
    std::cout << "range error: " << std::endl << std::endl;

    // hg sim
    std::cout << "hg sim:" << std::endl;
    std::string rel_path_sim_detail_hg = genPathSimDetail(section_id, "hg", hg_sim_version, query_type, tag);
    SimDetail sim_detail_hg(rel_path_sim_detail_hg);

    metric.dispRangeError(sim_detail_hg);

    dispHorizontalLine(25);
    // mm sim
    std::cout << "mm sim:" << std::endl;
    std::string rel_path_sim_detail_mm = genPathSimDetail(section_id, "mm", mm_sim_version, query_type, tag);
    SimDetail sim_detail_mm(rel_path_sim_detail_mm);

    metric.dispRangeError(sim_detail_mm);

    dispHorizontalLine();

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
