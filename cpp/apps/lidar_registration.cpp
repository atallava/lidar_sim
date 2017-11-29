#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

// load data, which i expect to be in the form of pose
// and associated pts at that pose
// then run the icp
// this should give a set of registered poses
// output the file
// i guess the section file format should already encode what is going on

// first thing to figure out is the scanning pattern in the real data

// then, given a section, because there are some subsampling etc done, output:
// section_xx_real
// section_pts_xx_real
// section_xx_sim
// section_pts_xx_sim

// the registration app should look at the section_xx_. each packet has associated timestamp, pts, and pose, so data is in desired format

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;

    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_id = 8;
    std::string rel_path_section = genRelPathSection(section_id);;
    SectionLoader section(rel_path_section);

    int step = 1;

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

// load section

// decide how many points to look at

// run the icp

// save the esimated poses to some file

// will need to viz also! damn
