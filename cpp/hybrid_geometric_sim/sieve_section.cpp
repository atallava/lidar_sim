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
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionModelSim.h>

using namespace lidar_sim;

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    // ss << "data" 
    //    << "/section_" << std::setw(2) << std::setfill('0') << section_id 
    //    << "_world_frame_subsampled.xyz";

    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame.xyz";

    return ss.str();
}

std::string genRelPathSievedPts(int section_id)
{
    std::ostringstream ss;
    ss << "data"
       << "/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_sieved.xyz";

    return ss.str();
}

// sieve means pts from poses in equispaced times in section
int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_id = 8;
    std::string rel_path_section = genRelPathSection(section_id);;
    SectionLoader section(rel_path_section);

    int n_poses_sieved = 500;
    int step = (int)(section.m_packet_timestamps.size())/(n_poses_sieved);

    std::vector<std::vector<double> > pts;
    for (size_t i = 0; i < section.m_packet_timestamps.size(); i += step)
    {
	double t = section.m_packet_timestamps[i];
	std::vector<std::vector<double> > t_pts = section.getPtsAtTime(t);
	pts.insert(pts.end(), t_pts.begin(), t_pts.end());
    }

    // write pts
    std::string rel_path_output = genRelPathSievedPts(section_id);
    writePtsToXYZFile(pts, rel_path_output);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
