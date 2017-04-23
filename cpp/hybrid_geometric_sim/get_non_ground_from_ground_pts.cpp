#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/TriangleModeler.h>

using namespace lidar_sim;

std::string genRelPathGroundPts(int section_id, int part_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/ground_segmentation/part_" << part_id << "_ground.asc";

    return ss.str();
}

std::string genRelPathNonGroundPts(int section_id, int part_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/ground_segmentation/part_" << part_id << "_non_ground.asc";

    return ss.str();
}

std::string genRelPathPts(int section_id, int part_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/ground_segmentation/part_" << part_id << ".asc";

    return ss.str();
}

int main(int argc, char **argv)
{
    int section_id = 4;
    int part_id = 4;

    std::string rel_path_pts = genRelPathPts(section_id, part_id);
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    std::string rel_path_ground_pts = genRelPathGroundPts(section_id, part_id);
    std::vector<std::vector<double> > ground_pts = loadPtsFromXYZFile(rel_path_ground_pts);

    std::vector<int> nbr_ids;
    std::tie(nbr_ids, std::ignore) = 
	nearestNeighbors(pts, ground_pts);

    std::vector<int> ground_flag = 
	genLogicalVecFromIds(nbr_ids, pts.size());

    std::vector<int> non_ground_flag = 
	negateLogicalVec(ground_flag);

    std::vector<std::vector<double> > non_ground_pts = 
	logicalSubsetArray(pts, non_ground_flag);

    std::string rel_path_non_ground_pts = genRelPathNonGroundPts(section_id, part_id);
    writePtsToXYZFile(non_ground_pts, rel_path_non_ground_pts);

    return(1);
}

