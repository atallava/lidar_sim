#include <iostream>
#include <iomanip>
#include <ctime>

// hacking boost filesystem bug
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <lidar_sim_classification/DataProcessingUtils.h>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

namespace lsc = lidar_sim_classification;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    std::string run_name = "barrels_2";

    // vehicle posn nodes
    std::string path_vehicle_posn_nodes = lsc::genPathVehiclePosnNodes(run_name);
    std::vector<std::vector<double> > vehicle_posn_nodes = loadArray(path_vehicle_posn_nodes, 3);

    // run ground pts
    std::string path_pts = lsc::genPathRunPtsGround(run_name);
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(path_pts);

    // the ground points were segmented from a centered cloud. add that back
    std::vector<double> pts_offset = lsc::getPtsCenteringOffset(run_name);
    for (size_t i = 0; i < pts.size(); ++i)
	for (size_t j = 0; j < pts[i].size(); ++j)
	    pts[i][j] += pts_offset[j];

    // nearest neighbor for pts in nodes
    std::vector<std::vector<int> > nn_ids;
    std::tie(nn_ids, std::ignore) = nearestNeighbors(vehicle_posn_nodes, pts, 1);
    
    // for each node, which pts are in it
    std::vector<std::vector<int> > node_pts_map(vehicle_posn_nodes.size(), 
						std::vector<int> ());
    
    for(size_t i = 0; i < pts.size(); ++i)
	node_pts_map[nn_ids[i][0]].push_back(i);
    
    // block size
    int pts_per_block = 1e5;

    // write blocks
    std::vector<std::vector<int> > block_node_ids;
    std::vector<int> current_block_node_ids(2,0);
    current_block_node_ids[0] = 0;

    int block_id = 1;

    // make dir where files will go
    std::string path_blocks_info_dir = lsc::genPathBlocksInfoDir(run_name);
    boost::filesystem::create_directories(path_blocks_info_dir);

    std::string path_block = lsc::genPathGroundBlock(run_name, block_id);
    std::ofstream block_file(path_block);
    std::cout << "Writing to: " << path_block << std::endl;
    
    std::string current_line;
    int pts_in_current_block = 0;
    for(size_t i = 0; i < node_pts_map.size(); ++i)
    {
	for(size_t j = 0; j < node_pts_map[i].size(); ++j)
	{
	    std::vector<double> pt = pts[node_pts_map[i][j]];
	    block_file << std::setprecision(15) << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
	}
	pts_in_current_block += node_pts_map[i].size();

	if (pts_in_current_block >= pts_per_block)
	{
	    if (i == node_pts_map.size()-1)
		break;

	    // block node ids
	    current_block_node_ids[1] = i;
	    block_node_ids.push_back(current_block_node_ids);
	    current_block_node_ids[0] = i+1;

	    // open new file
	    block_file.close();
	    block_id++;

	    path_block = lsc::genPathGroundBlock(run_name, block_id);
	    block_file.open(path_block);
	    std::cout << "Writing to: " << path_block << std::endl;

	    // reset counter
	    pts_in_current_block = 0;
	}
    }
    current_block_node_ids[1] = node_pts_map.size()-1;
    block_node_ids.push_back(current_block_node_ids);
    block_file.close();

    // write block node ids;
    std::string path_block_node_ids_ground = lsc::genPathBlockNodeIdsGround(run_name);
    writePtsToXYZFile(block_node_ids, path_block_node_ids_ground);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
    
    return(1);
}
