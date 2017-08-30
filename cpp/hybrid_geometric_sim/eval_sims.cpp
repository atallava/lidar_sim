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

std::string genRelPathSliceHgRealPts(int section_id, std::string query_type, int tag)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim"
       << "/" << query_type << "_real_pts";

    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";

    return ss.str();
}

std::string genRelPathHgSimPts(int section_id, std::string query_type, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim"
       << "/" << query_type << "_sim_pts";

    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";

    return ss.str();
}

std::string genRelPathHgSimDetail(int section_id, std::string query_type, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim"
       << "/" << query_type << "_sim_detail";

    if (tag == -1)
	ss << ".txt";
    else
	ss << "_" << tag << ".txt";

    return ss.str();
}

std::string genRelPathSliceNbrRealPts(int section_id, std::string query_type, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/nbr_sim"
       << "/" << query_type << "_real_pts";

    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";
    
    return ss.str();
}

std::string genRelPathNbrSimPts(int section_id, std::string query_type, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/nbr_sim"
       << "/" << query_type << "_sim_pts";

    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";

    return ss.str();
}

std::string genRelPathNbrSimDetail(int section_id, std::string query_type, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/nbr_sim"
       << "/" << query_type << "_sim_detail";

    if (tag == -1)
	ss << ".txt";
    else
	ss << "_" << tag << ".txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 8;
    int tag = -1;
    std::string query_type = "section";

    // error metric
    PtsError metric;

    // pcd error
    std::cout << "pcd error: " << std::endl << std::endl;

    // hg sim
    std::cout << "hg sim: " << std::endl;
    std::string rel_path_hg_real_pts = genRelPathSliceHgRealPts(section_id, query_type, tag);
    std::vector<std::vector<double> > hg_real_pts = loadPtsFromXYZFile(rel_path_hg_real_pts);
    std::string rel_path_hg_sim_pts = genRelPathHgSimPts(section_id, query_type, tag);
    std::vector<std::vector<double> > hg_sim_pts = loadPtsFromXYZFile(rel_path_hg_sim_pts);

    metric.dispPcdError(hg_real_pts, hg_sim_pts);

    dispHorizontalLine(25);

    // nbr sim
    std::cout << "nbr sim: " << std::endl;
    std::string rel_path_nbr_real_pts = genRelPathSliceNbrRealPts(section_id, query_type, tag);
    std::vector<std::vector<double> > nbr_real_pts = loadPtsFromXYZFile(rel_path_nbr_real_pts);
    std::string rel_path_nbr_sim_pts = genRelPathNbrSimPts(section_id, query_type, tag);
    std::vector<std::vector<double> > nbr_sim_pts = loadPtsFromXYZFile(rel_path_nbr_sim_pts);

    metric.dispPcdError(nbr_real_pts, nbr_sim_pts);

    dispHorizontalLine(25);
    // sanity check
    std::cout << std::endl;
    std::cout << "sanity check. nbr, hg real pts: " << std::endl;
    metric.dispPcdError(hg_real_pts, nbr_real_pts);

    dispHorizontalLine();

    // range error
    std::cout << "range error: " << std::endl << std::endl;

    // hg sim
    std::cout << "hg sim:" << std::endl;
    std::string rel_path_sim_detail_hg = genRelPathHgSimDetail(section_id, query_type, tag);
    SimDetail sim_detail_hg(rel_path_sim_detail_hg);

    metric.dispRangeError(sim_detail_hg);

    dispHorizontalLine(25);
    // nbr sim
    std::cout << "nbr sim:" << std::endl;
    std::string rel_path_sim_detail_nbr = genRelPathNbrSimDetail(section_id, query_type, tag);
    SimDetail sim_detail_nbr(rel_path_sim_detail_nbr);

    metric.dispRangeError(sim_detail_nbr);

    dispHorizontalLine();

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
