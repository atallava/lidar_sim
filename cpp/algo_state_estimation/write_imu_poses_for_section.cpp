#include <iostream>
#include <ctime>
#include <algorithm>

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/DataProcessingUtils.h>

using namespace lidar_sim;

std::string genRelPathOutput(int section_id)
{
    std::ostringstream ss;
    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/section_imu_poses.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();
    
    int section_id = 4;
    double section_start_time = 1403045911;
    double section_end_time = 1403046027;

    std::string path_poses_log = genPathPosesLog();
    PoseServer imu_pose_server(path_poses_log);

    size_t ids_pad = 20;
    size_t start_idx = imu_pose_server.getIndexOfNearestTime(section_start_time);
    start_idx = std::max<size_t>(0, start_idx - ids_pad);
    size_t end_idx = imu_pose_server.getIndexOfNearestTime(section_end_time);
    end_idx = std::min(end_idx + ids_pad, imu_pose_server.getNumLogs());

    // debug
    // std::cout << std::setprecision(20) << imu_pose_server.getTimeAtIndex(start_idx) 
    // 	      << " " << imu_pose_server.getTimeAtIndex(end_idx) << std::endl;

    // write out
    std::string rel_path_out = genRelPathOutput(section_id);
    std::ofstream file(rel_path_out);

    for (size_t i = start_idx; i <= end_idx; ++i)
    {
	double t = imu_pose_server.getTimeAtIndex(i);
	double intpart, fractpart;
	fractpart = modf(t, &intpart);
	int t_sec = (int)intpart;
	int t_nanosec = (int)(fractpart*1e9);

	Eigen::Matrix4d T = imu_pose_server.getTransfAtTime(t);

	std::ostringstream ss;
	ss << t_sec << " " << t_nanosec << " ";
	// transform written in column-major order
	for (size_t col = 0; col < 4; ++col)
	    for (size_t row = 0; row < 4; ++row)
		ss << T(row,col) << " ";

	ss << std::endl;
	file << ss.str();
    }

    file.close();
    std::cout << "Written imu poses for section to " << rel_path_out << std::endl;

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}

