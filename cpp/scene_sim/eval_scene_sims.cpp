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

std::string genRelPathSliceHgRealPts(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/slice_real_pts";
    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";

    return ss.str();
}

std::string genRelPathHgSimPts(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/slice_sim_pts";
    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";

    return ss.str();
}

std::string genRelPathHgSimDetail(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/slice_sim_detail";
    if (tag == -1)
	ss << ".txt";
    else
	ss << "_" << tag << ".txt";

    return ss.str();
}

std::string genRelPathSliceMmRealPts(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mm_sim/slice_real_pts";
    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";
    
    return ss.str();
}

std::string genRelPathMmSimPts(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mm_sim/slice_sim_pts";
    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";

    return ss.str();
}

std::string genRelPathMmSimDetail(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mm_sim/slice_sim_detail";
    if (tag == -1)
	ss << ".txt";
    else
	ss << "_" << tag << ".txt";

    return ss.str();
}
int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 4;
    int tag = 1;

    // error metric
    PtsError metric;

    // pcd error
    std::cout << "pcd error: " << std::endl;

    // hg sim
    std::string rel_path_hg_real_pts = genRelPathSliceHgRealPts(section_id, tag);
    std::vector<std::vector<double> > hg_real_pts = loadPtsFromXYZFile(rel_path_hg_real_pts);
    std::string rel_path_hg_sim_pts = genRelPathHgSimPts(section_id, tag);
    std::vector<std::vector<double> > hg_sim_pts = loadPtsFromXYZFile(rel_path_hg_sim_pts);

    std::cout << "hg sim: " << std::endl;
    metric.dispPcdError(hg_real_pts, hg_sim_pts);

    // mesh model sim
    std::string rel_path_mm_real_pts = genRelPathSliceMmRealPts(section_id, tag);
    std::vector<std::vector<double> > mm_real_pts = loadPtsFromXYZFile(rel_path_mm_real_pts);
    std::string rel_path_mm_sim_pts = genRelPathMmSimPts(section_id, tag);
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
    std::string rel_path_sim_detail_hg = genRelPathHgSimDetail(section_id, tag);
    SimDetail sim_detail_hg(rel_path_sim_detail_hg);

    std::cout << "hg sim:" << std::endl;
    metric.dispRangeError(sim_detail_hg);

    // mm sim
    std::string rel_path_sim_detail_mm = genRelPathMmSimDetail(section_id, tag);
    SimDetail sim_detail_mm(rel_path_sim_detail_mm);

    std::cout << "mm sim:" << std::endl;
    metric.dispRangeError(sim_detail_mm);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
