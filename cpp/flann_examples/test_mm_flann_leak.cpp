#include <tuple>
#include <ctime>
#include <sys/time.h>
#include <omp.h>

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

std::string genRelPathTriangles(int section_id, std::string sim_version, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/version_" << sim_version
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathObjectMeshesDir(int section_id, std::string sim_version)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mm_sim/version_" << sim_version;

    return ss.str();
}

std::string genRelPathObjectMesh(int section_id, std::string sim_version, int object_id)
{
   std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mm_sim/version_" << sim_version 
       << "/" << object_id << ".txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    typedef std::vector<std::vector<double> > Pts;
    typedef std::vector<std::vector<double> > Dirns;

    // load real packets
    int section_scans_id = 4;
    std::string scans_version = "260118"; // a small section so that valgrind doesn't take forever
    std::string rel_path_real_packets = 
	algo_state_est::genRelPathPacketsToProcess(section_scans_id, scans_version, "real");
    SectionLoader real_packets(rel_path_real_packets);

    // data for sim object
    // todo: remove this dependency on hg. it is confusing.
    // manual copying of files is better
    int section_hg_models_id = 4;
    std::string hg_sim_version = "080917";
    std::string path_hg_models_dir = genPathHgModelsDir(section_hg_models_id, hg_sim_version);

    // mm sim config
    std::string sim_type = "mm_sim";
    bool deterministic_sim = "false"; 
    int section_mm_models_id = 4;
    std::string mm_sim_version = "010218";
    std::vector<std::string> rel_path_object_meshes;
    std::string rel_path_object_meshes_dir = genRelPathObjectMeshesDir(section_mm_models_id, mm_sim_version);

    // pick a few objects only
    size_t n_objects = 10;
    std::vector<int> object_mesh_ids;
    for(size_t i = 1; i <= n_objects; ++i)
	object_mesh_ids.push_back(i);

    for(auto i : object_mesh_ids)
	rel_path_object_meshes.push_back(genRelPathObjectMesh(section_mm_models_id, mm_sim_version, i));

    // unfortunate that i need to load all triangles and the blocks info
    // that system needs to be destroyed
    // find ground triangle models for this section
    std::vector<std::string> rel_path_ground_triangle_model_blocks;
    std::vector<int> ground_triangle_model_block_ids = 
	getTriangleModelBlockIds(path_hg_models_dir, section_hg_models_id);
    for(auto i : ground_triangle_model_block_ids)
    	rel_path_ground_triangle_model_blocks.push_back(genRelPathTriangles(section_hg_models_id, hg_sim_version, i));

    // hg model blocks info
    std::string path_imu_posn_nodes = genPathImuPosnNodes(section_hg_models_id);
    std::string path_block_node_ids_ground = genPathBlockNodeIdsGround(section_hg_models_id);

    // obtain ray information per packet for sim
    // currently this is serial
    size_t n_packets = real_packets.m_packet_ids.size();
    n_packets = 5;
    std::vector<Pts> real_pts_per_packet;
    std::vector<std::vector<double> > ray_origin_per_packet;
    std::vector<Dirns> ray_dirns_per_packet;
    SimDetail sim_detail; 
    sim_detail.m_sim_pts_all.resize(n_packets);
    sim_detail.m_sim_hit_flags.resize(n_packets);

    LaserCalibParams laser_calib_params;

    // pose, ray servers
    std::string path_poses_log = genPathPosesLog();
    PoseServer imu_pose_server(path_poses_log);
    RayDirnServer ray_dirn_server;

    for (size_t i = 0; i < n_packets; ++i)
    {
	int packet_id = real_packets.m_packet_ids[i];
	double t = real_packets.m_packet_timestamps[i];
	
	// pose, ray origin
	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, laser_calib_params);

	// packet pts
	Pts this_packet_real_pts = real_packets.getPtsAtTime(t);
	real_pts_per_packet.push_back(this_packet_real_pts); 
	
	// ray dirns
	std::vector<double> this_packet_ray_pitches;
	std::vector<double> this_packet_ray_yaws;
	std::vector<std::vector<double> > this_real_pts_all;
	std::vector<int> this_real_hit_flag;
	std::tie(this_packet_ray_pitches, this_packet_ray_yaws, this_real_pts_all, this_real_hit_flag)
	    = ray_dirn_server.fitDetailToPts(ray_origin, real_pts_per_packet[i]);
	std::vector<std::vector<double> > ray_dirns = calcRayDirnsFromSph(this_packet_ray_pitches, this_packet_ray_yaws);

	// store for parallel sim
	ray_origin_per_packet.push_back(ray_origin);
	ray_dirns_per_packet.push_back(ray_dirns);

	// add to sim detail
	// packet details
	sim_detail.m_packet_ids.push_back(packet_id);
	sim_detail.m_packet_timestamps.push_back(t);
	// ray origin
	sim_detail.m_ray_origins.push_back(ray_origin);
	// pitches
	sim_detail.m_ray_pitches.push_back(this_packet_ray_pitches);
	// yaws
	sim_detail.m_ray_yaws.push_back(this_packet_ray_yaws);
	// real pts all
	sim_detail.m_real_pts_all.push_back(this_real_pts_all);
	// real hit flag
	sim_detail.m_real_hit_flags.push_back(this_real_hit_flag);

	// resize sim detail data for parallel sim
	size_t n_rays = ray_dirns.size();
	sim_detail.m_sim_pts_all[i].resize(n_rays, std::vector<double> (3));
	sim_detail.m_sim_hit_flags[i].resize(n_rays);
    }

    // create sim object
    MeshModelSim sim;
    sim.loadTriangleModelBlocks(rel_path_ground_triangle_model_blocks); // semantically this is ground
    sim.loadObjectMeshes(rel_path_object_meshes);
    sim.setDeterministicSim(deterministic_sim);
    sim.loadBlockInfo(path_imu_posn_nodes, path_block_node_ids_ground);

    // sim
    for(size_t i = 0; i < n_packets; ++i)
    {	
	sim.simPtsGivenRays(
		ray_origin_per_packet[i], ray_dirns_per_packet[i]);
    }

    struct timeval end_time;
    gettimeofday(&end_time, NULL);

    double elapsed_time = ((end_time.tv_sec  - start_time.tv_sec) * 1000000u + 
			   end_time.tv_usec - start_time.tv_usec) / 1.e6;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
