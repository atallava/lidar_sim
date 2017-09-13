#include <tuple>
#include <ctime>

#include <vtkProperty.h>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/GeometricSegmenter.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 1;

    // load section
    std::string path_section = genPathSection(section_id);
    SectionLoader section(path_section);

    // pose server
    std::string path_poses_log = genPathPosesLog();
    PoseServer imu_pose_server(path_poses_log);

    // imu posn nodes
    std::vector<std::vector<double> > imu_posn_nodes;
    double dt = 1; // in s

    double prev_t = section.m_packet_timestamps[0]-dt;
    for(size_t i = 0; i < section.m_packet_timestamps.size(); ++i)
    {
    	double t = section.m_packet_timestamps[i];
    	if ( (t-prev_t) >= dt )
    	{
    	    std::vector<double> imu_posn = posnFromImuPose(imu_pose_server.getPoseAtTime(t));
    	    imu_posn_nodes.push_back(imu_posn);
    	    prev_t = t;
    	}
    }

    // viz these nodes
    RangeDataVizer vizer;
    vizer.vizPts(imu_posn_nodes);

    // write out
    writePtsToXYZFile(imu_posn_nodes, genPathImuPosnNodes(section_id));

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
    
    return(1);
}
