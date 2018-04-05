#include <tuple>
#include <utility>
#include <ctime>
#include <sys/time.h>
#include <omp.h>

// hacking boost filesystem bug
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <lidar_sim_classification/DataProcessingUtils.h>
#include <lidar_sim_classification/ModelingUtils.h>
#include <lidar_sim_classification/ObbUtils.h>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionModelSim.h>
#include <lidar_sim/SimDetail.h>
#include <lidar_sim/RayDirnServer.h>
#include <lidar_sim/AlgoStateEstUtils.h>

using namespace lidar_sim;

namespace lsc = lidar_sim_classification;

int main(int argc, char **argv)
{
    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    typedef std::vector<std::vector<double> > Pts;
    // typedef std::vector<std::vector<double> > Dirns;

    std::string run_name = "barrels_1";

    // data for sim object
    std::string sim_type = "hg_sim";
    bool deterministic_sim = "false";
    std::string sim_version = "260218";

    // find ellipsoid models for this section
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    std::vector<int> ellipsoid_model_block_ids = 
	lsc::getEllipsoidModelBlockIds(run_name, sim_version);
    for(auto i : ellipsoid_model_block_ids) {
	std::string rel_path_ellipsoids = lsc::genRelPathEllipsoids(run_name, sim_version, i);
    	rel_path_ellipsoid_model_blocks.push_back(rel_path_ellipsoids);
    }

    // find triangle models for this section
    std::vector<std::string> rel_path_triangle_model_blocks;
    std::vector<int> triangle_model_block_ids = 
	lsc::getTriangleModelBlockIds(run_name, sim_version);
    for(auto i : triangle_model_block_ids) {
	std::string rel_path_triangles = lsc::genRelPathTriangles(run_name, sim_version, i);
    	rel_path_triangle_model_blocks.push_back(rel_path_triangles);
    }

    // model blocks info
    std::string path_imu_posn_nodes = lsc::genPathVehiclePosnNodes(run_name);
    std::string path_block_node_ids_ground = lsc::genPathBlockNodeIdsGround(run_name);
    std::string path_block_node_ids_non_ground = lsc::genPathBlockNodeIdsNonGround(run_name);

    // rays for sim
    lsc::RaysForSimColln rays_for_sim_colln = lsc::calcRaysForSim(run_name);
    size_t n_packets_to_sim = rays_for_sim_colln.ray_origin_per_packet.size();

    // preallocate space
    std::vector<Pts> sim_returns_per_packet;
    sim_returns_per_packet.resize(n_packets_to_sim);
    std::vector<std::vector<int> > sim_hit_flag_per_packet;
    sim_hit_flag_per_packet.resize(n_packets_to_sim);
    int n_returns_per_packet = 32; // fixed
    for (size_t i = 0; i < n_packets_to_sim; ++i)
    {
    	sim_returns_per_packet[i].resize(n_returns_per_packet, std::vector<double> (3));
	sim_hit_flag_per_packet[i].resize(n_returns_per_packet);
    }

    // todo: how many threads?    
    int num_threads = 1;
#pragma omp parallel num_threads (num_threads) 
    {
	// create sim object
	SectionModelSim sim;
	{
#pragma omp critical (load_object_data)	
	    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
	    sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);
	    sim.setDeterministicSim(deterministic_sim);
	    sim.loadBlockInfo(path_imu_posn_nodes, path_block_node_ids_ground, path_block_node_ids_non_ground);
	}

	// sim. loop over packets
#pragma omp for
	for(size_t i = 0; i < n_packets_to_sim; ++i)
	{	
	    std::tie(
		sim_returns_per_packet[i], sim_hit_flag_per_packet[i]) = 
		sim.simPtsGivenRays(rays_for_sim_colln.ray_origin_per_packet[i], rays_for_sim_colln.ray_dirns_per_packet[i]);
	}

    }   // omp parallel ends

    // write

    // load obbs, open detail file hangles
    // scene barrels
    std::vector<lsc::Obb> scene_barrels_obbs;
    std::vector<std::shared_ptr<std::ofstream> > files_barrel_detail;

    // make dir
    std::string path_barrel_detail_dir = lsc::genPathSceneBarrelDetailDir(run_name, sim_type, sim_version);
    boost::filesystem::create_directories(path_barrel_detail_dir);

    int n_scene_barrels = lsc::getNSceneBarrels(run_name);
    for (size_t i = 0; i < (size_t)n_scene_barrels; ++i)
    {
	std::string path_obb = lsc::genPathSceneBarrelObb(run_name, i);
	scene_barrels_obbs.push_back(
	    lsc::loadObb(path_obb));
	std::string path_detail = lsc::genPathSceneBarrelDetail(run_name, sim_type, sim_version, i);
	std::shared_ptr<std::ofstream> file(new std::ofstream);
	file->open(path_detail.c_str());
	files_barrel_detail.push_back(file);
    }

    // load negatives obbs
    std::vector<lsc::Obb> scene_negatives_obbs;
    std::vector<std::shared_ptr<std::ofstream> > files_negative_detail;
    // make dir
    std::string path_negative_detail_dir = lsc::genPathSceneNegativeDetailDir(run_name, sim_type, sim_version);
    boost::filesystem::create_directories(path_negative_detail_dir);

    int n_scene_negatives = lsc::getNSceneNegatives(run_name);
    for (size_t i = 0; i < (size_t)n_scene_negatives; ++i)
    {
    	std::string path_obb = lsc::genPathSceneNegativeObb(run_name, i);
    	scene_negatives_obbs.push_back(
    	    lsc::loadObb(path_obb));
    	std::string path_detail = lsc::genPathSceneNegativeDetail(run_name, sim_type, sim_version, i);
	std::shared_ptr<std::ofstream> file(new std::ofstream);
	file->open(path_detail.c_str());
    	files_negative_detail.push_back(file);
    }

    // loop over packets
    for (size_t i = 0; i < n_packets_to_sim; ++i)
    {
	std::string object_type = rays_for_sim_colln.object_type[i];
	int scan_id = rays_for_sim_colln.scan_ids[i];
	int packet_id = rays_for_sim_colln.packet_ids[i];
	int object_id = rays_for_sim_colln.object_ids[i];

	Pts packet_returns = sim_returns_per_packet[i];
	std::vector<int> hit_flag = sim_hit_flag_per_packet[i];
	// loop over returns
	for (size_t j = 0; j < hit_flag.size(); ++j)
	{
	    if (hit_flag[j])
	    {
		std::vector<double> pt = packet_returns[j];
		if (strcmp(object_type.c_str(), "scene_barrel") == 0)
		{
		    lsc::Obb obb = scene_barrels_obbs[object_id];
		    if (checkPointInObb(obb, pt))
		    {
			std::ostringstream oss;
			oss << scan_id << " " << packet_id  << " " 
			    << std::setprecision(15) << pt[0] << " " << pt[1] << " " << pt[2] 
			    << std::endl;
			std::string line = oss.str();
			files_barrel_detail[object_id]->write(line.c_str(), line.size());
		    }
		}
		else if (strcmp(object_type.c_str(), "scene_barrel") == 0)
		{
		    lsc::Obb obb = scene_negatives_obbs[object_id];
		    if (checkPointInObb(obb, pt))
		    {
			std::ostringstream oss;
			oss << scan_id << " " << packet_id  << " " 
			    << std::setprecision(15) << pt[0] << " " << pt[1] << " " << pt[2] 
			    << std::endl;
			std::string line = oss.str();
			files_negative_detail[object_id]->write(line.c_str(), line.size());
		    }
		}
		else {
		    std::stringstream ss_err_msg;
		    ss_err_msg << "unknown object type " << object_type;
		    throw std::runtime_error(ss_err_msg.str().c_str());
		}

	    }
	}
    }

    // close all files
    for (size_t i = 0; i < files_barrel_detail.size(); ++i)
	files_barrel_detail[i]->close();
    for (size_t i = 0; i < files_negative_detail.size(); ++i)
	files_negative_detail[i]->close();

    std::cout << "Written barrel details to " << path_barrel_detail_dir << std::endl;
    std::cout << "Written negative details to " << path_negative_detail_dir << std::endl;

    struct timeval end_time;
    gettimeofday(&end_time, NULL);

    double elapsed_time = ((end_time.tv_sec  - start_time.tv_sec) * 1000000u + 
			   end_time.tv_usec - start_time.tv_usec) / 1.e6;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
