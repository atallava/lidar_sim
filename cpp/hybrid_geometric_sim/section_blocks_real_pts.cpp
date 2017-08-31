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

std::string genRelPathBlocksRealPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/blocks_real_pts.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_id = 3;
    std::string rel_path_section = genPathSection(section_id);
    SectionLoader section(rel_path_section);

    std::vector<std::vector<double> > real_pts;
    std::vector<int> non_ground_blocks {16,17,18,19};
    std::vector<int> ground_blocks {};
    int num_nbrs = 1;
    
    // ground block pts
    for (auto block_id : ground_blocks)
    {
	std::string rel_path_pts = genPathGroundBlockPts(section_id, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);

	std::vector<std::vector<int> > section_nbr_pt_ids; 

	std::tie(section_nbr_pt_ids, std::ignore) = 
	    nearestNeighbors(section.m_pts, block_pts, num_nbrs);

	for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	    for(size_t j = 0; j < (size_t)num_nbrs; ++j)
	    {
		int idx = section_nbr_pt_ids[i][j];
		real_pts.push_back(section.m_pts[idx]);
	    }
    }

    // non ground block pts
    for (auto block_id : non_ground_blocks)
    {
	std::string rel_path_pts = genPathNonGroundBlockPts(section_id, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);

	std::vector<std::vector<int> > section_nbr_pt_ids; 

	std::tie(section_nbr_pt_ids, std::ignore) = 
	    nearestNeighbors(section.m_pts, block_pts, num_nbrs);

	for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	    for(size_t j = 0; j < (size_t)num_nbrs; ++j)
	    {
		int idx = section_nbr_pt_ids[i][j];
		real_pts.push_back(section.m_pts[idx]);
	    }
    }

    // write out
    std::string rel_path_pts = genRelPathBlocksRealPts(section_id);
    writePtsToXYZFile(real_pts, rel_path_pts);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
