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

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    ss << "data/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

std::string genRelPathSectionPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load real pts
    int section_id_val = 8;
    std::string rel_path_pts = genRelPathSectionPts(section_id_val);
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // load sim pts
    std::string rel_path_pts_sim = "data/rim_stretch_validation_sim.xyz";
    std::vector<std::vector<double> > pts_sim = loadPtsFromXYZFile(rel_path_pts_sim);

    // load sim pts
    std::string rel_path_pts_sim_nbr = "data/rim_stretch_validation_sim_nn.xyz";
    std::vector<std::vector<double> > pts_sim_nn = loadPtsFromXYZFile(rel_path_pts_sim_nbr);

    // error metric
    // PtsError metric;

    // double error_sim = metric.calcError(pts, pts_sim);
    // std::cout << "sim error: " << error_sim << std::endl;

    // double error_sim_nn = metric.calcError(pts, pts_sim_nn);
    // std::cout << "sim nn error: " << error_sim_nn << std::endl;

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
