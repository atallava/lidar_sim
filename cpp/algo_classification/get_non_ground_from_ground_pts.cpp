#include <string>
#include <iostream>
#include <vector>
#include <sys/time.h>

#include <lidar_sim_classification/DataProcessingUtils.h>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    std::string run_name = "barrels_1";
    int part_id = 3;

    std::string path_pts = lidar_sim_classification::genPathGascolaLogsDir() + "/" + run_name 
    	+ "/ground_segmentation/part_" + std::to_string(part_id) + ".asc";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(path_pts);

    std::string path_ground_pts = lidar_sim_classification::genPathGascolaLogsDir() + "/" + run_name 
    	+ "/ground_segmentation/part_" + std::to_string(part_id) + "_ground.asc";
    std::vector<std::vector<double> > ground_pts = loadPtsFromXYZFile(path_ground_pts);

    std::vector<int> nbr_ids;
    std::tie(nbr_ids, std::ignore) = 
    	nearestNeighbors(pts, ground_pts);

    std::vector<int> ground_flag = 
    	genLogicalVecFromIds(nbr_ids, pts.size());

    std::vector<int> non_ground_flag = 
    	negateLogicalVec(ground_flag);

    std::vector<std::vector<double> > non_ground_pts = 
    	logicalSubsetArray(pts, non_ground_flag);

    std::string path_non_ground_pts = lidar_sim_classification::genPathGascolaLogsDir() + "/" + run_name 
    	+ "/ground_segmentation/part_" + std::to_string(part_id) + "_non_ground.asc";
    writePtsToXYZFile(non_ground_pts, path_non_ground_pts);

    struct timeval end_time;
    gettimeofday(&end_time, NULL);
    double elapsed_time = ((end_time.tv_sec  - start_time.tv_sec) * 1000000u + 
			   end_time.tv_usec - start_time.tv_usec) / 1.e6;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

