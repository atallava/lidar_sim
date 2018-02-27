#include <string>
#include <iostream>
#include <vector>
#include <ctime>

// hacking boost filesystem bug
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <lidar_sim_classification/DataProcessingUtils.h>
#include <lidar_sim_classification/VehiclePoseServer.h>
#include <lidar_sim_classification/ModelingUtils.h>

#include <lidar_sim/TriangleModeler.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/SectionLoader.h>

using namespace lidar_sim;

namespace lsc = lidar_sim_classification;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    std::string run_name = "barrels_2";
    std::vector<int> block_ids = lsc::getGroundBlockIds(run_name);

    // run as section
    int run_section_skip_exp = 2;
    std::string path_section = lsc::genPathRunAsSection(run_name, run_section_skip_exp);
    SectionLoader section(path_section);

    // vehicle pose server
    std::string path_poses = lsc::genPathVehiclePoses(run_name);
    lsc::VehiclePoseServer pose_server(path_poses);

    // blocks info
    std::string path_vehicle_posn_nodes = lsc::genPathVehiclePosnNodes(run_name);
    std::vector<std::vector<double> > vehicle_posn_nodes = loadArray(path_vehicle_posn_nodes, 3);
    std::string path_block_node_ids_ground = lsc::genPathBlockNodeIdsGround(run_name);
    std::vector<std::vector<int> > block_node_ids_ground = 
    	doubleToIntArray(loadArray(path_block_node_ids_ground, 2));

    // mkdirs for sim models
    std::string hg_sim_version = "260218"; // todo: currently hand-specified
    std::string path_sim_models_dir = lsc::genPathSimModelsDir(run_name, "hg_sim", hg_sim_version);
    boost::filesystem::create_directories(path_sim_models_dir);

    // loop over blocks
    for(size_t i = 0; i < block_ids.size(); ++i)
    {
    	std::cout << "processing block " << i << "..." << std::endl;

    	// model each block
    	int block_id = block_ids[i];
    	std::string path_pts = lsc::genPathGroundBlock(run_name, block_id);
    	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(path_pts);

    	TriangleModeler modeler;
    	modeler.setDebugFlag(1);
    	modeler.createTriangleModels(path_pts);
	
    	// get rays to process for hit prob calc
    	int num_nbrs = 1;
    	std::vector<std::vector<int> > section_nbr_pt_ids; 

    	std::tie(section_nbr_pt_ids, std::ignore) = nearestNeighbors(section.m_pts, block_pts, num_nbrs);

	std::vector<std::vector<double> > ray_origins;
	std::vector<std::vector<double> > ray_endpoints;
	for (size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
    	    for(size_t j = 0; j < section_nbr_pt_ids[i].size(); ++j)
	    {
		int id = section_nbr_pt_ids[i][j];
		double t = section.m_pt_timestamps[id];
		std::vector<double> this_pt = section.m_pts[id];
		std::vector<double> ray_origin = lsc::getLaserPosnAtTime(pose_server, t);

		ray_origins.push_back(ray_origin);
		ray_endpoints.push_back(this_pt);
	    }

    	modeler.calcHitProb(ray_origins, ray_endpoints);

    	// write out triangles
    	std::string path_triangles = 
	    lsc::genPathGroundTriangles(run_name, "hg_sim", hg_sim_version, block_id);
    	modeler.writeTrianglesToFile(path_triangles);
    }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

