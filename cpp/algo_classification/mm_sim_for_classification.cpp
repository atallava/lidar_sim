#include <tuple>
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
#include <lidar_sim/MeshModelSim.h>
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
    // todo: remove this dependency on hg. it is confusing.
    // manual copying of files is better
    std::string hg_sim_version = "260218";

    // find object meshes
    std::string sim_type = "mm_sim";
    bool deterministic_sim = "false"; 
    std::string mm_sim_version = "260218";
    std::vector<std::string> rel_path_object_meshes;
    std::vector<int> object_mesh_ids = lsc::getMmObjectMeshIds(run_name, mm_sim_version);
    for(auto i : object_mesh_ids) 
    {
	std::string rel_path_mesh = lsc::genRelPathMmObjectMesh(run_name, mm_sim_version, i);
	rel_path_object_meshes.push_back(rel_path_mesh);
    }

    // barrels for this scene
    // add them to rel path object meshes
    size_t n_scene_barrels = lsc::getNSceneBarrels(run_name);
    for (size_t i = 0; i < n_scene_barrels; ++i) 
    {
	int barrel_id = i;
	std::string path_barrel_posed = lsc::genPathBarrelPosedMeshModel(run_name, barrel_id);
	rel_path_object_meshes.push_back(path_barrel_posed);
    }

    // find ground triangle models for this scene
    std::vector<std::string> rel_path_ground_triangle_model_blocks;
    std::vector<int> ground_triangle_model_block_ids = 
	lsc::getTriangleModelBlockIds(run_name, hg_sim_version);
    for(auto i : ground_triangle_model_block_ids) {
	std::string rel_path_triangles = lsc::genRelPathTriangles(run_name, hg_sim_version, i);
    	rel_path_ground_triangle_model_blocks.push_back(rel_path_triangles);
    }

    // hg model blocks info
    std::string path_imu_posn_nodes = lsc::genPathVehiclePosnNodes(run_name);
    std::string path_block_node_ids_ground = lsc::genPathBlockNodeIdsGround(run_name);

    // rays for sim
    bool use_reduced_assignment = true;
    lsc::PacketsForSimColln packets_for_sim_colln = lsc::calcPacketsForSim(run_name, use_reduced_assignment);
    size_t n_packets_to_sim = packets_for_sim_colln.ray_origin_per_packet.size();
    // n_packets_to_sim = 10; // todo: modify

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
    int num_threads = 6;
#pragma omp parallel num_threads (num_threads) 
    {
	// create sim object
	MeshModelSim sim;
	{
#pragma omp critical (load_object_data)	
	    sim.loadTriangleModelBlocks(rel_path_ground_triangle_model_blocks); // semantically this is ground
	    sim.loadObjectMeshes(rel_path_object_meshes);
	    sim.setDeterministicSim(deterministic_sim);
	    sim.loadBlockInfo(path_imu_posn_nodes, path_block_node_ids_ground);
	}

	// sim. loop over packets
#pragma omp for
	for(size_t i = 0; i < n_packets_to_sim; ++i)
	{	
	    std::tie(
		sim_returns_per_packet[i], sim_hit_flag_per_packet[i]) = 
		sim.simPtsGivenRays(packets_for_sim_colln.ray_origin_per_packet[i], packets_for_sim_colln.ray_dirns_per_packet[i]);
	}

    }   // omp parallel ends

    // write

    // load obbs, open detail file handles
    // scene barrels
    std::vector<lsc::Obb> scene_barrels_obbs;
    std::vector<std::shared_ptr<std::ofstream> > files_barrel_detail;

    // make dir
    std::string path_barrel_detail_dir = lsc::genPathSceneBarrelDetailDir(run_name, sim_type, mm_sim_version);
    boost::filesystem::create_directories(path_barrel_detail_dir);

    for (size_t i = 0; i < (size_t)n_scene_barrels; ++i)
    {
	std::string path_obb = lsc::genPathSceneBarrelObb(run_name, i);
	scene_barrels_obbs.push_back(
	    lsc::loadObb(path_obb));
	std::string path_detail = lsc::genPathSceneBarrelDetail(run_name, sim_type, mm_sim_version, i);
	std::shared_ptr<std::ofstream> file(new std::ofstream);
	file->open(path_detail.c_str());
	files_barrel_detail.push_back(file);
    }

    // load negatives obbs
    std::vector<lsc::Obb> scene_negatives_obbs;
    std::vector<std::shared_ptr<std::ofstream> > files_negative_detail;
    // make dir
    std::string path_negative_detail_dir = lsc::genPathSceneNegativeDetailDir(run_name, sim_type, mm_sim_version);
    boost::filesystem::create_directories(path_negative_detail_dir);

    size_t n_scene_negatives = lsc::getNSceneNegatives(run_name);
    for (size_t i = 0; i < (size_t)n_scene_negatives; ++i)
    {
    	std::string path_obb = lsc::genPathSceneNegativeObb(run_name, i);
    	scene_negatives_obbs.push_back(
    	    lsc::loadObb(path_obb));
    	std::string path_detail = lsc::genPathSceneNegativeDetail(run_name, sim_type, mm_sim_version, i);
	std::shared_ptr<std::ofstream> file(new std::ofstream);
	file->open(path_detail.c_str());
    	files_negative_detail.push_back(file);
    }

    // loop over packets
    for (size_t i = 0; i < n_packets_to_sim; ++i)
    {
	std::string object_type = packets_for_sim_colln.object_types[i];
	int scan_id = packets_for_sim_colln.scan_ids[i];
	int packet_id = packets_for_sim_colln.packet_ids[i];
	int object_id = packets_for_sim_colln.object_ids[i];

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
		else if (strcmp(object_type.c_str(), "scene_negative") == 0)
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

    // write sim detail
    // barrels
    std::string path_sim_detail_for_barrels_dir = lsc::genPathSimDetailForObjectsDir(run_name, sim_type, mm_sim_version, "scene_barrel");
    boost::filesystem::create_directories(path_sim_detail_for_barrels_dir);
    for (size_t i = 0; i < n_scene_barrels; ++i)
    {
	std::string rel_path_sim_detail = lsc::genPathSimDetailForObject(run_name, sim_type, mm_sim_version, "scene_barrel", i);
	std::vector<int> packet_ids_for_object = packets_for_sim_colln.getPacketIdsForObject("scene_barrel", i);
	lsc::writeSimDetailForObject(rel_path_sim_detail, packets_for_sim_colln, sim_returns_per_packet, sim_hit_flag_per_packet, packet_ids_for_object);
    }
    std::cout << "Written sim detail for barrels to " << path_sim_detail_for_barrels_dir << std::endl;
    // negatives
    std::string path_sim_detail_for_negatives_dir = lsc::genPathSimDetailForObjectsDir(run_name, sim_type, mm_sim_version, "scene_negative");
    boost::filesystem::create_directories(path_sim_detail_for_negatives_dir);
    for (size_t i = 0; i < n_scene_negatives; ++i)
    {
	std::string rel_path_sim_detail = lsc::genPathSimDetailForObject(run_name, sim_type, mm_sim_version, "scene_negative", i);
	std::vector<int> packet_ids_for_object = packets_for_sim_colln.getPacketIdsForObject("scene_negative", i);
	lsc::writeSimDetailForObject(rel_path_sim_detail, packets_for_sim_colln, sim_returns_per_packet, sim_hit_flag_per_packet, packet_ids_for_object);
    }
    std::cout << "Written sim detail for negatives to " << path_sim_detail_for_negatives_dir << std::endl;

    struct timeval end_time;
    gettimeofday(&end_time, NULL);

    double elapsed_time = ((end_time.tv_sec  - start_time.tv_sec) * 1000000u + 
			   end_time.tv_usec - start_time.tv_usec) / 1.e6;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
