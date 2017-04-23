#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/TriangleModeler.h>
#include <lidar_sim/PtsError.h>

using namespace lidar_sim;

std::string genRelPathBlock(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground.xyz";

    return ss.str();
}

std::string genRelPathTriangles(int param_id)
{
    std::ostringstream ss;
    ss << "data/analyze_mesh/triangles_" << std::setw(2) << std::setfill('0') << param_id
       << ".txt";

    return ss.str();
}

std::string genRelPathRealPts(int param_id)
{
    std::ostringstream ss;
    ss << "data/analyze_mesh/real_pts_" << std::setw(2) << std::setfill('0') << param_id
       << ".xyz";

    return ss.str();
}

std::string genRelPathSimPts(int param_id)
{
    std::ostringstream ss;
    ss << "data/analyze_mesh/sim_pts_" << std::setw(2) << std::setfill('0') << param_id
       << ".xyz";

    return ss.str();
}

std::string genRelPathParamVec()
{
    std::ostringstream ss;
    ss << "data/analyze_mesh/param_vec.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    int section_id = 3;
    int block_id = 4;
    
    std::cout << "processing block " << block_id << "..." << std::endl;

    // load section
    std::string rel_path_section = "data/section_03_world_frame_subsampled.xyz";
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // load block pts 
    std::string rel_path_block_pts = genRelPathBlock(section_id, block_id);
    std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_block_pts);
    
    // section pts to process for hit prob
    int num_nbrs = 1;
    std::vector<std::vector<int> > section_nbr_pt_ids; 
    std::tie(section_nbr_pt_ids, std::ignore) = nearestNeighbors(section.m_pts, block_pts, num_nbrs);
    std::vector<std::vector<double> > section_pts_to_process;
    std::vector<int> section_pt_ids_to_process;
    for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	for(size_t j = 0; j < section_nbr_pt_ids[i].size(); ++j)
	{
	    int id = section_nbr_pt_ids[i][j];
	    section_pt_ids_to_process.push_back(id);
	    section_pts_to_process.push_back(
		section.m_pts[id]);
	}

    // param vec
    double min_param_val = 0.1;
    double max_param_val = 1.5;
    double n_params = 25;
    double param_step = (max_param_val-min_param_val)/n_params;
    std::vector<double> param_vec;
    double param = min_param_val;
    while (param < max_param_val)
    {
	param_vec.push_back(param);
	param += param_step;
    }

    // write param vec
    std::string rel_path_param_vec = genRelPathParamVec();
    writeVecToFile(param_vec, rel_path_param_vec);

    std::vector<double> comp_time_vec;
    std::vector<double> sym_loss_mean_vec;

    // loop over params
    for(size_t i = 0; i < param_vec.size(); ++i)
    {    
	std::cout << "parameter: " << i << std::endl;
	clock_t start_time = clock();

        // create mesh models
	TriangleModeler modeler;
	modeler.setDebugFlag(1);

	// triangles
	modeler.createTriangleModels(rel_path_block_pts);

	// calc hit prob
	std::cout << "calculating hit prob..." << std::endl;
	modeler.calcHitProb(section, section_pt_ids_to_process, imu_pose_server);

	// write triangle models to file
	std::string rel_path_triangles;
	rel_path_triangles = genRelPathTriangles(i);
	modeler.writeTrianglesToFile(rel_path_triangles);
    
	// sim
	std::cout << "simming..." << std::endl;
	TriangleModelSim sim;
	sim.m_max_residual_for_hit = param_vec[i];
	sim.loadTriangleModels(rel_path_triangles);
	// loop over section pts
	std::vector<std::vector<double> > real_pts;
	std::vector<std::vector<double> > sim_pts;
	for(size_t j = 0; j < section_pt_ids_to_process.size(); ++j)
	{
	    size_t id = section_pt_ids_to_process[j];
	    double t = section.m_pt_timestamps[id];

	    std::vector<double> real_pt = section.m_pts[id];
	    std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
	    std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, sim.m_laser_calib_params);
	    std::vector<double> ray_dirn;
	    std::tie(ray_dirn, std::ignore) = calcRayDirn(ray_origin, real_pt);
	    std::vector<double> sim_pt;
	    int hit_flag;
	    std::tie(sim_pt, hit_flag) = sim.simPtGivenRay(ray_origin, ray_dirn);

	    // log
	    real_pts.push_back(real_pt);
	    if (hit_flag)
		sim_pts.push_back(sim_pt);
	}

	// write real pts
	std::string rel_path_real_pts = genRelPathRealPts(i);
	writePtsToXYZFile(real_pts, rel_path_real_pts);

	std::string rel_path_sim_pts = genRelPathSimPts(i);
	writePtsToXYZFile(sim_pts, rel_path_sim_pts);

	// error 
	std::cout << "calculating error..." << std::endl;
	PtsError metric;
	double asym_loss_real_sim_mean,  asym_loss_real_sim_var;
	std::tie(asym_loss_real_sim_mean, asym_loss_real_sim_var) = metric.calcAsymmetricError(real_pts, sim_pts);
	std::cout << "asym_loss_real_sim_mean: " << asym_loss_real_sim_mean << " asym_loss_real_sim_var: " << asym_loss_real_sim_var << std::endl;

	double asym_loss_sim_real_mean,  asym_loss_sim_real_var;
	std::tie(asym_loss_sim_real_mean, asym_loss_sim_real_var) = metric.calcAsymmetricError(sim_pts, real_pts);
	std::cout << "asym_loss_sim_real_mean: " << asym_loss_sim_real_mean << " asym_loss_sim_real_var: " << asym_loss_sim_real_var << std::endl;

	double sym_loss_mean = (asym_loss_real_sim_mean + asym_loss_sim_real_mean)/2.0;
	std::cout << "sym_loss_mean: " << sym_loss_mean << std::endl;
	sym_loss_mean_vec.push_back(sym_loss_mean);

	// compuation time
	double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
	std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	comp_time_vec.push_back(elapsed_time);
	
	std::cout << std::endl << std::endl;
    }

    std::string rel_path_sym_loss_mean_vec = "data/analyze_mesh/sym_loss_mean_vec.txt";
    writeVecToFile(sym_loss_mean_vec, rel_path_sym_loss_mean_vec);
    std::string rel_path_comp_time_vec = "data/analyze_mesh/comp_time_vec.txt";
    writeVecToFile(comp_time_vec, rel_path_comp_time_vec);

    return(1);
}

