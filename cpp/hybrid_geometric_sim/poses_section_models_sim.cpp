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

std::string genRelPathSimPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_sim.xyz";

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

    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_models_id);
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(section_models_id);
    std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_models_id);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    int n_poses_to_sim = 1;

    // sim
    // loop over packets
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    int packet_array_step = std::floor(section.m_packet_ids.size()/ n_poses_to_sim);
    for(size_t i = 0; i < section.m_packet_ids.size(); i += packet_array_step)
    {
	double t = section.m_packet_timestamps[i];

	// pose, ray origin
	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);

	// packet pts
	std::vector<std::vector<double> > this_pts = section.getPtsAtTime(t);
	
	// ray dirns
	std::vector<std::vector<double> > ray_dirns  = calcRayDirns(ray_origin, this_pts);

	// simulate 
	std::vector<std::vector<double> > this_sim_pts;
	std::vector<int> this_hit_flag;
	std::tie(this_sim_pts, this_hit_flag) = sim.simPtsGivenRays(ray_origin, ray_dirns); 

	// add to big list
	sim_pts_all.insert(sim_pts_all.end(), this_sim_pts.begin(), this_sim_pts.end());
	hit_flag.insert(hit_flag.end(), this_hit_flag.begin(), this_hit_flag.end());
    }

    // // get poses
    // std::vector<std::vector<double> > imu_poses;
    // double t_max = section.m_pt_timestamps.back();
    // double t_min = section.m_pt_timestamps[0];
    // double dt = (t_max-t_min)/n_poses_to_sim;

    // double t = t_min;
    // while(t < t_max)
    // {
    // 	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    // 	imu_poses.push_back(imu_pose);
    // 	t += dt;
    // }
    
    // // simulate
    // std::vector<std::vector<double> > sim_pts_all;
    // std::vector<int> hit_flag;
    // std::tie(sim_pts_all, hit_flag) = sim.simPtsGivenPoses(imu_poses);
    
    // weed out non-hits
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);

    // write sim pts
    std::string rel_path_output = genRelPathSimPts(section_sim_id);;
    writePtsToXYZFile(sim_pts, rel_path_output);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
