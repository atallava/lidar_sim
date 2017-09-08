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

    // todo: delete

    // error metric
    PtsError metric;
    std::string path1 = "/usr0/home/atallav1/lidar_sim/cpp/data/sections/section_08/hg_sim/version_310817/section_real_pts.xyz";
    std::string path2 = "/usr0/home/atallav1/lidar_sim/cpp/data/sections/section_08/hg_sim/version_310817/section_sim_pts.xyz";
    std::vector<std::vector<double> > pts1 = loadPtsFromXYZFile(path1);
    std::vector<std::vector<double> > pts2 = loadPtsFromXYZFile(path2);
    metric.dispPcdError(pts1,pts2);

    // todo: clear
    std::string path3 = "/usr0/home/atallav1/lidar_sim/cpp/data/sections/section_08/hg_sim/version_310817/section_real_pts.xyz";
    std::string path4 = "/usr0/home/atallav1/lidar_sim/cpp/data/sections/section_08/hg_sim/version_310817/section_sim_pts.xyz";
    std::vector<std::vector<double> > pts3 = loadPtsFromXYZFile(path3);
    std::vector<std::vector<double> > pts4 = loadPtsFromXYZFile(path4);
    metric.dispPcdError(pts3,pts4);

    std::exit(0);

    int section_id = 8;
    int tag = -1;
    std::string query_type = "section";
    // todo: uncomment
    std::string hg_sim_version = "310817";
    std::string nbr_sim_version = "250417";

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

    // todo: uncomment
    // // nbr sim
    // std::cout << "nbr sim, version " << nbr_sim_version << " : " << std::endl;
    // std::string rel_path_nbr_real_pts =  genPathRealPtsRef(section_id, "nbr", nbr_sim_version, 
    // 							   query_type, tag);
    // std::vector<std::vector<double> > nbr_real_pts = loadPtsFromXYZFile(rel_path_nbr_real_pts);
    // std::string rel_path_nbr_sim_pts =   genPathSimPts(section_id, "nbr", nbr_sim_version, 
    // 						       query_type, tag);
    // std::vector<std::vector<double> > nbr_sim_pts = loadPtsFromXYZFile(rel_path_nbr_sim_pts);

    // metric.dispPcdError(nbr_real_pts, nbr_sim_pts);

    // dispHorizontalLine(25);
    // // sanity check
    // std::cout << std::endl;
    // std::cout << "sanity check. nbr, hg real pts: " << std::endl;
    // metric.dispPcdError(hg_real_pts, nbr_real_pts);

    // dispHorizontalLine();

    // // range error
    // std::cout << "range error: " << std::endl << std::endl;
    // int disp_range_error = 1;

    // // hg sim
    // std::cout << "hg sim:" << std::endl;
    // std::string rel_path_sim_detail_hg = genPathSimDetail(section_id, "hg", hg_sim_version, query_type, tag);
    // SimDetail sim_detail_hg(rel_path_sim_detail_hg);

    // metric.calcRangeError(sim_detail_hg, disp_range_error);

    // dispHorizontalLine(25);
    // // nbr sim
    // std::cout << "nbr sim:" << std::endl;
    // std::string rel_path_sim_detail_nbr = genPathSimDetail(section_id, "nbr", nbr_sim_version, query_type, tag);
    // SimDetail sim_detail_nbr(rel_path_sim_detail_nbr);
    
    // metric.calcRangeError(sim_detail_nbr, disp_range_error);

    // dispHorizontalLine();

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<std::pair<std::string, std::string> > sims_to_eval; // sim type, sim version
    sims_to_eval.push_back(std::make_pair("hg", "310817"));
    // sims_to_eval.push_back(std::make_pair("nbr", "250417")); // todo: uncomment

    size_t n_sims_to_eval = sims_to_eval.size();

    for (size_t i = 0; i < n_sims_to_eval; i++) {
	std::string this_sim_type = sims_to_eval[i].first;
	std::string this_sim_version = sims_to_eval[i].second;

	std::cout << "sim type: " << this_sim_type 
		  << ", sim version: " << this_sim_version << std::endl;
	dispHorizontalLine(10);

	std::cout << "pcd error: " << std::endl;
	std::string path_real_pts = genPathRealPtsRef(section_id, this_sim_type, this_sim_version,
						     query_type, tag);
	std::vector<std::vector<double> > real_pts = loadPtsFromXYZFile(path_real_pts);
	std::string path_sim_pts =  genPathSimPts(section_id, this_sim_type, this_sim_version, 
						  query_type, tag);
	std::vector<std::vector<double> > sim_pts = loadPtsFromXYZFile(path_sim_pts);
	metric.dispPcdError(real_pts, sim_pts);
	dispHorizontalLine(10);

	// todo: uncomment
	// std::cout << "range error: " << std::endl << std::endl;
	// int disp_range_error = 1;
	// // hg sim
	// std::string path_sim_detail = genPathSimDetail(section_id, this_sim_type, this_sim_version, 
	// 					       query_type, tag);
	// SimDetail sim_detail(path_sim_detail);
	// metric.calcRangeError(sim_detail, disp_range_error);
	// dispHorizontalLine(50);
    }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
