#include <vector>
#include <ctime>

#include <lidar_sim/DataProcessingUtils.h>

#include <lidar_sim_classification/DataProcessingUtils.h>
#include <lidar_sim_classification/VehiclePoseServer.h>

using namespace lidar_sim;

namespace lsc = lidar_sim_classification;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    std::string run_name = "barrels_2";

    // pose server
    std::string path_poses = lsc::genPathVehiclePoses(run_name);
    lsc::VehiclePoseServer pose_server(path_poses);

    // posn nodes
    std::vector<std::vector<double> > vehicle_posn_nodes;
    double dt = 1; // in s

    size_t n_poses = pose_server.getNPoses();
    double t = pose_server.getTimeAtIndex(0);
    double t_end = pose_server.getTimeAtIndex(n_poses-1);

    while (t < t_end)
    {
	std::vector<double> vehicle_pose = pose_server.getPoseAtTime(t);
	std::vector<double> vehicle_posn (3);
	for (size_t i = 0; i < 3; ++i)
	    vehicle_posn[i] = vehicle_pose[i];
	vehicle_posn_nodes.push_back(vehicle_posn);
	t += dt;
    }

    // write out
    writePtsToXYZFile(vehicle_posn_nodes, lsc::genPathVehiclePosnNodes(run_name));

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
    
    return(1);
}
