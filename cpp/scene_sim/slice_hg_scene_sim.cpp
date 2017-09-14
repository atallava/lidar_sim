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
#include <lidar_sim/SimDetail.h>
#include <lidar_sim/RayDirnServer.h>

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

std::string genRelPathEllipsoids(int section_id, std::string sim_version, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/version_" << sim_version
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

std::string genRelPathQueriedBlocks(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/slice_sim_queried_blocks";
    if (tag == -1)
	ss << ".txt";
    else
	ss << "_" << tag << ".txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_sim_id = 1;
    std::string path_section = genPathSection(section_sim_id);
    SectionLoader section(path_section);

    // sim object
    int section_models_id = 1;
    std::string sim_version = "130917";
    std::string path_models_dir = genPathHgModelsDir(section_models_id, sim_version);

    // find ellipsoid models for this section
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    std::vector<int> ellipsoid_model_block_ids = 
	getEllipsoidModelBlockIds(path_models_dir, section_models_id);
    for(auto i : ellipsoid_model_block_ids)
    	rel_path_ellipsoid_model_blocks.push_back(genRelPathEllipsoids(section_models_id, sim_version, i));

    // find triangle models for this section
    std::vector<std::string> rel_path_triangle_model_blocks;
    std::vector<int> triangle_model_block_ids = 
	getTriangleModelBlockIds(path_models_dir, section_models_id);
    for(auto i : triangle_model_block_ids)
    	rel_path_triangle_model_blocks.push_back(genRelPathTriangles(section_models_id, sim_version, i));

    // create sim object
    SectionModelSim sim;
    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
    sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);

    sim.setDeterministicSim(false);

    std::string path_imu_posn_nodes = genPathImuPosnNodes(section_models_id);
    std::string path_block_node_ids_ground = genPathBlockNodeIdsGround(section_models_id);
    std::string path_block_node_ids_non_ground = genPathBlockNodeIdsNonGround(section_models_id);

    sim.loadBlockInfo(path_imu_posn_nodes, path_block_node_ids_ground, path_block_node_ids_non_ground);

    // pose server
    std::string path_poses_log = genPathPosesLog();
    PoseServer imu_pose_server(path_poses_log);

    // slice ids
    size_t packet_id_sim_start, packet_id_sim_end;
    packet_id_sim_start = 0; 
    packet_id_sim_end = section.m_packet_ids.size();

    RayDirnServer ray_dirn_server;

    // sim
    // loop over packets
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    std::vector<std::vector<double> > real_pts;
    std::vector<int> ellipsoid_blocks_queried;
    std::vector<int> triangle_blocks_queried;
    SimDetail sim_detail;
    size_t packet_array_step = 20; // todo: check!

    for(size_t i = packet_id_sim_start; 
	i < packet_id_sim_end; i += packet_array_step)
    {
    	double t = section.m_packet_timestamps[i];

    	// pose, ray origin
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);

	// packet pts
    	std::vector<std::vector<double> > this_real_pts = section.getPtsAtTime(t);

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
	
	// blocks queried for this pose
	std::vector<int> this_triangle_blocks_queried = 
	    sim.getPosnTriangleBlockMembership(ray_origin);
	triangle_blocks_queried.insert(triangle_blocks_queried.begin(),
					this_triangle_blocks_queried.begin(), this_triangle_blocks_queried.end());
	std::vector<int> this_ellipsoid_blocks_queried = 
	    sim.getPosnEllipsoidBlockMembership(ray_origin);
	ellipsoid_blocks_queried.insert(ellipsoid_blocks_queried.begin(),
					this_ellipsoid_blocks_queried.begin(), this_ellipsoid_blocks_queried.end());

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
    }

    // retain unique block ids
    ellipsoid_blocks_queried = getUniqueSortedVec(ellipsoid_blocks_queried);
    triangle_blocks_queried = getUniqueSortedVec(triangle_blocks_queried);
    
    // weed out non-hits in sim pts
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);

    // write real pts
    int tag = -1;
    std::string sim_type = "hg";
    std::string query_type = "slice";
    std::string path_real_pts = genPathRealPtsRef(section_sim_id, sim_type, sim_version, 
						  query_type, tag);
    writePtsToXYZFile(real_pts, path_real_pts);

    // write sim pts
    std::string path_sim_pts = genPathSimPts(section_sim_id, sim_type, sim_version, 
					     query_type, tag);
    writePtsToXYZFile(sim_pts, path_sim_pts);

    // write sim detail
    std::string path_sim_detail = genPathSimDetail(section_sim_id, sim_type, sim_version, 
						    query_type, tag);
    sim_detail.save(path_sim_detail);

    // write queried blocks
    std::string rel_path_queried_blocks = genRelPathQueriedBlocks(section_sim_id, tag); 
    writeQueriedBlocks(rel_path_queried_blocks, triangle_blocks_queried, ellipsoid_blocks_queried);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
