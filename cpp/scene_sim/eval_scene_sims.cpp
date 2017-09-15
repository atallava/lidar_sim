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

    int section_id = 1;
    std::string query_type = "slice";
    int tag = -1;

    // todo: make script accept these as input
    std::vector<std::pair<std::string, std::string> > sims_to_eval; // sim type, sim version
    sims_to_eval.push_back(std::make_pair("hg", "130917"));
    sims_to_eval.push_back(std::make_pair("mm", "130917")); 

    PtsError metric;
    size_t n_sims_to_eval = sims_to_eval.size();
    for (size_t i = 0; i < n_sims_to_eval; i++) {
	std::string this_sim_type = sims_to_eval[i].first;
	std::string this_sim_version = sims_to_eval[i].second;

	std::cout << "sim type: " << this_sim_type 
		  << ", sim version: " << this_sim_version << std::endl;
	dispHorizontalLine(10);

	std::cout << "pcd error: " << std::endl << std::endl;
	std::string path_real_pts = genPathRealPtsRef(section_id, this_sim_type, this_sim_version,
						     query_type, tag);
	std::vector<std::vector<double> > real_pts = loadPtsFromXYZFile(path_real_pts);
	std::string path_sim_pts =  genPathSimPts(section_id, this_sim_type, this_sim_version, 
						  query_type, tag);
	std::vector<std::vector<double> > sim_pts = loadPtsFromXYZFile(path_sim_pts);
	metric.dispPcdError(real_pts, sim_pts);
	dispHorizontalLine(10);

	std::cout << "range error: " << std::endl << std::endl;
	int disp_range_error = 1;
	std::string path_sim_detail = genPathSimDetail(section_id, this_sim_type, this_sim_version, 
						       query_type, tag);
	SimDetail sim_detail(path_sim_detail);
	metric.calcRangeError(sim_detail, disp_range_error);
	dispHorizontalLine(50);
    }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
