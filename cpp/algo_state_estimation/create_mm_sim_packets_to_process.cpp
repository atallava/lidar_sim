#include <tuple>
#include <ctime>

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
    clock_t start_time = clock();

    // load real packets
    int section_scans_id = 4;
    std::string scans_version = "260118"; 
    std::string rel_path_real_packets = 
	algo_state_est::genRelPathPacketsToProcess(section_scans_id, scans_version, "real");
    SectionLoader real_packets(rel_path_real_packets);

    // sim object
    // todo: remove this dependency on hg. it is confusing.
    // manual copying of files is better
    int section_hg_models_id = 4;
    std::string hg_sim_version = "080917";
    std::string path_hg_models_dir = genPathHgModelsDir(section_hg_models_id, hg_sim_version);

    // find object meshes
    std::string sim_type = "mm_sim";
    int section_mm_models_id = 4;
    std::string mm_sim_version = "010218";
    std::vector<std::string> rel_path_object_meshes;
    std::string rel_path_object_meshes_dir = genRelPathObjectMeshesDir(section_mm_models_id, mm_sim_version);
    std::vector<int> object_mesh_ids = 
    	getObjectMeshIds(rel_path_object_meshes_dir);

    for(auto i : object_mesh_ids)
	rel_path_object_meshes.push_back(genRelPathObjectMesh(section_mm_models_id, mm_sim_version, i));

    // find ground triangle models for this section
    std::vector<std::string> rel_path_ground_triangle_model_blocks;
    std::vector<int> ground_triangle_model_block_ids = 
	getTriangleModelBlockIds(path_hg_models_dir, section_hg_models_id);
    for(auto i : ground_triangle_model_block_ids)
    	rel_path_ground_triangle_model_blocks.push_back(genRelPathTriangles(section_hg_models_id, hg_sim_version, i));

    // create sim object
    MeshModelSim sim;
    sim.loadTriangleModelBlocks(rel_path_ground_triangle_model_blocks); // semantically this is ground
    sim.loadObjectMeshes(rel_path_object_meshes);

    sim.setDeterministicSim(false);

    // blocks info
    std::string path_imu_posn_nodes = genPathImuPosnNodes(section_hg_models_id);
    std::string path_block_node_ids_ground = genPathBlockNodeIdsGround(section_hg_models_id);
    sim.loadBlockInfo(path_imu_posn_nodes, path_block_node_ids_ground);

    // pose, ray servers
    std::string path_poses_log = genPathPosesLog();
    PoseServer imu_pose_server(path_poses_log);

    RayDirnServer ray_dirn_server;

    // sim packets file
    algo_state_est::mkdirsForPacketsToProcess(section_scans_id, scans_version, 
					      sim_type, mm_sim_version);
    std::string rel_path_sim_packets = 
	algo_state_est::genRelPathPacketsToProcess(section_scans_id, scans_version, sim_type, mm_sim_version);
    std::ofstream file_sim_packets(rel_path_sim_packets);
    
    // sim
    // loop over packets
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    std::vector<std::vector<double> > real_pts;
    std::vector<int> objects_queried;
    std::vector<int> ground_triangle_blocks_queried;
    SimDetail sim_detail;
    size_t n_packets = real_packets.m_packet_ids.size();

    // todo: change/ delete me!
    n_packets = 10;

    for(size_t i = 0; i < n_packets; ++i)
    {
    	double t = real_packets.m_packet_timestamps[i];

    	// pose, ray origin
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);

    	// packet pts
    	std::vector<std::vector<double> > this_real_pts = real_packets.getPtsAtTime(t);

    	// add to big list of real pts
    	real_pts.insert(real_pts.end(), this_real_pts.begin(), this_real_pts.end());

    	// ray dirns
	std::vector<double> this_ray_pitches;
	std::vector<double> this_ray_yaws;
	std::vector<std::vector<double> > this_real_pts_all;
	std::vector<int> this_real_hit_flag;
	std::tie(this_ray_pitches, this_ray_yaws, this_real_pts_all, this_real_hit_flag)
	    = ray_dirn_server.fitDetailToPts(ray_origin, this_real_pts);

	std::vector<std::vector<double> > ray_dirns = calcRayDirnsFromSph(this_ray_pitches, this_ray_yaws);

    	// simulate 
    	std::vector<std::vector<double> > this_sim_pts_all;
    	std::vector<int> this_sim_hit_flag;
    	std::tie(this_sim_pts_all, this_sim_hit_flag) = sim.simPtsGivenRays(ray_origin, ray_dirns); 

    	// add to big list of sim pts
    	sim_pts_all.insert(sim_pts_all.end(), this_sim_pts_all.begin(), this_sim_pts_all.end());
    	hit_flag.insert(hit_flag.end(), this_sim_hit_flag.begin(), this_sim_hit_flag.end());

	// add to sim detail
	// ray origin
	sim_detail.m_ray_origins.push_back(ray_origin);
	// pitches
	sim_detail.m_ray_pitches.push_back(this_ray_pitches);
	// yaws
	sim_detail.m_ray_yaws.push_back(this_ray_yaws);
	// real pts all
	sim_detail.m_real_pts_all.push_back(this_real_pts_all);
	// real hit flag
	sim_detail.m_real_hit_flags.push_back(this_real_hit_flag);
	// sim pts all
	sim_detail.m_sim_pts_all.push_back(this_sim_pts_all);
	// sim hit flag
	sim_detail.m_sim_hit_flags.push_back(this_sim_hit_flag);

	// write hits to sim packets
	int packet_id = real_packets.m_packet_ids[i];
	double intpart, fractpart;
	fractpart = modf(t, &intpart);
	int t_sec = (int)intpart;
	int t_nanosec = (int)(fractpart*1e9);
	for (size_t j = 0; j < this_sim_pts_all.size(); ++j)
	{
	    if (!this_sim_hit_flag[j]) 
		continue;

	    std::vector<double> pt = this_sim_pts_all[j];
	    std::ostringstream ss;
	    ss << packet_id << " " << t_sec << " " << t_nanosec << " " 
	       << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
	    file_sim_packets << ss.str();
	}
    }

    std::cout << "Written packets to process to: " << rel_path_sim_packets  << std::endl;
    file_sim_packets.close();

    // write real pts
    std::string rel_path_real_pts = 
	algo_state_est::genRelPathRealPtsRef(section_scans_id, scans_version, sim_type, mm_sim_version);
    writePtsToXYZFile(real_pts, rel_path_real_pts);

    // write sim pts
    std::string rel_path_sim_pts = 
	algo_state_est::genRelPathSimPts(section_scans_id, scans_version, sim_type, mm_sim_version);
    // weed out non-hits in sim pts
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);
    writePtsToXYZFile(sim_pts, rel_path_sim_pts);

    // write sim detail
    std::string rel_path_sim_detail = 
	algo_state_est::genRelPathSimDetail(section_scans_id, scans_version, sim_type, mm_sim_version);
    sim_detail.save(rel_path_sim_detail);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
