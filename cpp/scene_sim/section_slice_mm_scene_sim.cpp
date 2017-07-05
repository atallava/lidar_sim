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
#include <lidar_sim/MeshModelSim.h>
#include <lidar_sim/SimDetail.h>

using namespace lidar_sim;

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathObjectMeshesDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mesh_model_sim";

    return ss.str();
}

std::string genRelPathObjectMesh(int section_id, int object_id)
{
   std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mesh_model_sim/" << object_id << ".txt";

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
       << "/hg_sim/block_node_ids_ground.txt";

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

std::string genRelPathSliceRealPts(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mesh_model_sim/slice_real_pts";
    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";
    
    return ss.str();
}

std::string genRelPathSimPts(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mesh_model_sim/slice_sim_pts";
    if (tag == -1)
	ss << ".xyz";
    else
	ss << "_" << tag << ".xyz";

    return ss.str();
}

std::string genRelPathSimDetail(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mesh_model_sim/slice_sim_detail";
    if (tag == -1)
	ss << ".txt";
    else
	ss << "_" << tag << ".txt";

    return ss.str();
}

std::string genRelPathQueriedBlocks(int section_id, int tag = -1)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mesh_model_sim/slice_sim_queried_blocks";
    if (tag == -1)
	ss << ".txt";
    else
	ss << "_" << tag << ".txt";

    return ss.str();
}

std::string genRelPathHgModelsDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/hg_sim";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load section
    int section_sim_id = 4;
    std::string rel_path_section = genRelPathSection(section_sim_id);
    SectionLoader section(rel_path_section);

    // sim object
    int section_hg_models_id = 4;
    std::string rel_path_hg_models_dir = genRelPathHgModelsDir(section_hg_models_id);

    // find object meshes
    std::vector<std::string> rel_path_object_meshes;
    std::string rel_path_object_meshes_dir = genRelPathObjectMeshesDir(section_sim_id);
    std::vector<int> object_mesh_ids = 
    	getObjectMeshIds(rel_path_object_meshes_dir);

    for(auto i : object_mesh_ids)
	rel_path_object_meshes.push_back(genRelPathObjectMesh(section_sim_id, i));

    // find triangle models for this section
    std::vector<std::string> rel_path_ground_triangle_model_blocks;
    std::vector<int> ground_triangle_model_block_ids = 
	getTriangleModelBlockIds(rel_path_hg_models_dir, section_hg_models_id);
    for(auto i : ground_triangle_model_block_ids)
    	rel_path_ground_triangle_model_blocks.push_back(genRelPathTriangles(section_hg_models_id, i));

    // create sim object
    MeshModelSim sim;
    sim.loadTriangleModelBlocks(rel_path_ground_triangle_model_blocks); // semantically this is ground
    sim.loadObjectMeshes(rel_path_object_meshes);

    sim.setDeterministicSim(false);

    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_hg_models_id);
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(section_hg_models_id);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // slice ids
    size_t packet_id_sim_start, packet_id_sim_end;
    packet_id_sim_start = 0;
    packet_id_sim_end = section.m_packet_timestamps.size();

    // sim
    // loop over packets
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    std::vector<std::vector<double> > real_pts;
    std::vector<int> objects_queried;
    std::vector<int> ground_triangle_blocks_queried;
    SimDetail sim_detail;
    size_t packet_array_step = 3; 

    for(size_t i = packet_id_sim_start; 
    	i < packet_id_sim_end; i += packet_array_step)
    {
    	double t = section.m_packet_timestamps[i];

    	// pose, ray origin
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);

    	// packet pts
    	std::vector<std::vector<double> > this_pts = section.getPtsAtTime(t);

    	// add to big list of real pts
    	real_pts.insert(real_pts.end(), this_pts.begin(), this_pts.end());

    	// ray dirns
    	// here is where you could alternately get directions from laser intrinsics
    	std::vector<std::vector<double> > ray_dirns  = calcRayDirns(ray_origin, this_pts);

    	// blocks queried for this pose
    	std::vector<int> this_ground_triangle_blocks_queried = 
    	    sim.getPosnTriangleBlockMembership(ray_origin);
    	ground_triangle_blocks_queried.insert(ground_triangle_blocks_queried.begin(),
    					this_ground_triangle_blocks_queried.begin(), this_ground_triangle_blocks_queried.end());
    	std::vector<int> this_objects_queried = 
    	    sim.calcObjectIdsForSim(ray_origin, ray_dirns);
    	objects_queried.insert(objects_queried.begin(),
    					this_objects_queried.begin(), this_objects_queried.end());

    	// debug
    	// std::cout << "packet id: " << i << ". ray origin: " << std::endl;
    	// dispVec(ray_origin);
    	// dispVec(this_ground_triangle_blocks_queried);
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
	sim_detail.m_ray_origins.push_back(ray_origin);
	// real pts
	sim_detail.m_real_pts.push_back(this_pts);
	// sim pts
	sim_detail.m_sim_pts.push_back(this_sim_pts);
	// hit flag
	sim_detail.m_hit_flags.push_back(this_hit_flag);
    }

    // retain unique block ids
    objects_queried = getUniqueSortedVec(objects_queried);
    ground_triangle_blocks_queried = getUniqueSortedVec(ground_triangle_blocks_queried);
    
    // weed out non-hits
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);

    int tag = 2;

    // write real pts
    std::string rel_path_real_pts = genRelPathSliceRealPts(section_sim_id, tag);
    writePtsToXYZFile(real_pts, rel_path_real_pts);

    // write sim pts
    std::string rel_path_sim_pts = genRelPathSimPts(section_sim_id, tag);
    writePtsToXYZFile(sim_pts, rel_path_sim_pts);

    // write sim detail
    std::string rel_path_sim_detail = genRelPathSimDetail(section_sim_id); 
    sim_detail.save(rel_path_sim_detail);

    // write queried blocks
    std::string rel_path_queried_blocks = genRelPathQueriedBlocks(section_sim_id, tag); 
    writeQueriedBlocks(rel_path_queried_blocks, ground_triangle_blocks_queried, objects_queried);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
