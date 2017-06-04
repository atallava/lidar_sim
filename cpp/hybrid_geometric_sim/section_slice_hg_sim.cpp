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

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathEllipsoids(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

std::string genRelPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

    return ss.str();
}

std::string genRelPathBlockNodeIdsGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/block_node_ids_ground.txt";

    return ss.str();
}

std::string genRelPathBlockNodeIdsNonGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/block_node_ids_non_ground.txt";

    return ss.str();
}

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

std::string genRelPathSliceRealPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/slice_real_pts.xyz";

    return ss.str();
}

std::string genRelPathSimPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/slice_hg_sim_pts.xyz";

    return ss.str();
}

std::string genRelPathSimDetail(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/slice_hg_sim_detail.txt";

    return ss.str();
}

std::string genRelPathQueriedBlocks(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/slice_hg_sim_queried_blocks.txt";

    return ss.str();
}

std::string genRelPathModelsDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id;

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_sim_id = 8;
    std::string rel_path_section = genRelPathSection(section_sim_id);
    SectionLoader section(rel_path_section);

    // sim object
    int section_models_id = 3;
    std::string rel_path_models_dir = genRelPathModelsDir(section_models_id);;

    // find ellipsoid models for this section
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    std::vector<int> ellipsoid_model_block_ids = 
	getEllipsoidModelBlockIds(rel_path_models_dir, section_models_id);
    for(auto i : ellipsoid_model_block_ids)
    	rel_path_ellipsoid_model_blocks.push_back(genRelPathEllipsoids(section_models_id, i));

    // find triangle models for this section
    std::vector<std::string> rel_path_triangle_model_blocks;
    std::vector<int> triangle_model_block_ids = 
	getTriangleModelBlockIds(rel_path_models_dir, section_models_id);
    for(auto i : triangle_model_block_ids)
    	rel_path_triangle_model_blocks.push_back(genRelPathTriangles(section_models_id, i));

    // create sim object
    SectionModelSim sim;
    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
    sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);

    sim.setDeterministicSim(false);

    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_models_id);
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(section_models_id);
    std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_models_id);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // slice ids
    size_t packet_id_sim_start, packet_id_sim_end;
    packet_id_sim_start = 50000; // 40000
    packet_id_sim_end = packet_id_sim_start + 10000; // 20000

    // sim
    // loop over packets
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    std::vector<std::vector<double> > real_pts;
    std::vector<int> ellipsoid_blocks_queried;
    std::vector<int> triangle_blocks_queried;
    std::vector<std::vector<double> > sim_detail;
    size_t packet_array_step = 10; // 100
    for(size_t i = packet_id_sim_start; 
	i < packet_id_sim_end; i += packet_array_step)
    {
    	double t = section.m_packet_timestamps[i];
	std::vector<double> this_sim_detail;

    	// pose, ray origin
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);

	// add ray origin to sim detail
	this_sim_detail.insert(this_sim_detail.end(), ray_origin.begin(), ray_origin.end());

	// packet pts
    	std::vector<std::vector<double> > this_pts = section.getPtsAtTime(t);

    	// add to big list of real pts
    	real_pts.insert(real_pts.end(), this_pts.begin(), this_pts.end());

	// add all pts to sim detail
	for(size_t j = 0; j < this_pts.size(); ++j)
	    this_sim_detail.insert(this_sim_detail.end(), 
				   this_pts[j].begin(), this_pts[j].end());
	
    	// ray dirns
	// here is where you could alternately get directions from laser intrinsics
    	std::vector<std::vector<double> > ray_dirns  = calcRayDirns(ray_origin, this_pts);

	// blocks queried for this pose
	std::vector<int> this_triangle_blocks_queried = 
	    sim.getPosnTriangleBlockMembership(ray_origin);
	triangle_blocks_queried.insert(triangle_blocks_queried.begin(),
					this_triangle_blocks_queried.begin(), this_triangle_blocks_queried.end());
	std::vector<int> this_ellipsoid_blocks_queried = 
	    sim.getPosnEllipsoidBlockMembership(ray_origin);
	ellipsoid_blocks_queried.insert(ellipsoid_blocks_queried.begin(),
					this_ellipsoid_blocks_queried.begin(), this_ellipsoid_blocks_queried.end());

	// debug
	// std::cout << "packet id: " << i << ". ray origin: " << std::endl;
	// dispVec(ray_origin);
	// dispVec(this_triangle_blocks_queried);
	// dispVec(this_ellipsoid_blocks_queried);

    	// simulate 
    	std::vector<std::vector<double> > this_sim_pts;
    	std::vector<int> this_hit_flag;
    	std::tie(this_sim_pts, this_hit_flag) = sim.simPtsGivenRays(ray_origin, ray_dirns); 

    	// add to big list of sim pts
    	sim_pts_all.insert(sim_pts_all.end(), this_sim_pts.begin(), this_sim_pts.end());
    	hit_flag.insert(hit_flag.end(), this_hit_flag.begin(), this_hit_flag.end());

	// add to sim detail
	// ray origin
	sim_detail.push_back(ray_origin);
	// real pts
	sim_detail.push_back(
	    convertArrayToVec(this_pts));	
	// sim pts
	sim_detail.push_back(
	    convertArrayToVec(this_sim_pts));
	// hit flag
	sim_detail.push_back(
	    convertIntVecToDoubleVec(this_hit_flag));
    }

    // retain unique block ids
    ellipsoid_blocks_queried = getUniqueSortedVec(ellipsoid_blocks_queried);
    triangle_blocks_queried = getUniqueSortedVec(triangle_blocks_queried);
    
    // weed out non-hits
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);

    // write real pts
    std::string rel_path_real_pts = genRelPathSliceRealPts(section_sim_id);
    writePtsToXYZFile(real_pts, rel_path_real_pts);

    // write sim pts
    std::string rel_path_sim_pts = genRelPathSimPts(section_sim_id);
    writePtsToXYZFile(sim_pts, rel_path_sim_pts);

    // write sim detail
    std::string rel_path_sim_detail = genRelPathSimDetail(section_sim_id); 
    writePtsToXYZFile(sim_detail, rel_path_sim_detail);

    // write queried blocks
    std::string rel_path_queried_blocks = genRelPathQueriedBlocks(section_sim_id); "data/hg_sim_queried_blocks.txt";
    writeQueriedBlocks(rel_path_queried_blocks, triangle_blocks_queried, ellipsoid_blocks_queried);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
