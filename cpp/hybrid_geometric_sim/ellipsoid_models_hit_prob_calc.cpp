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
#include <lidar_sim/EllipsoidModelSim.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    std::cout << "loading data..." << std::endl;
    // load ellipsoids
    std::string rel_path_ellipsoid_models = "data/ellipsoid_models.txt";
    EllipsoidModels ellipsoid_models = 
	loadEllipsoidModelsFromFile(rel_path_ellipsoid_models);
    
    // load section
    std::string rel_path_section = "data/section_03_world_frame_subsampled_timed.xyz";
    SectionLoader section(rel_path_section);
    
    // pts from xyz
    std::string rel_path_xyz = "data/rim_stretch_veg_train.asc";
    std::vector<std::vector<double> > train_pts = loadPtsFromXYZFile(rel_path_xyz);
    
    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);
    
    std::cout << "calculating section ids to process..." << std::endl;
    // section ids to process
    std::vector<int> section_pt_ids_to_process;
    std::tie(section_pt_ids_to_process, std::ignore) = nearestNeighbors(section.m_pts, train_pts);

    // sim object
    const LaserCalibParams laser_calib_params;
    EllipsoidModelSim sim;
    sim.setEllipsoidModels(ellipsoid_models);
    sim.setLaserCalibParams(laser_calib_params);

    size_t n_ellipsoids = ellipsoid_models.size();
    std::vector<int> ellipsoid_hit_count_prior(n_ellipsoids, 1);
    std::vector<int> ellipsoid_miss_count_prior(n_ellipsoids, 0);

    std::vector<int> ellipsoid_hit_count = ellipsoid_hit_count_prior;
    std::vector<int> ellipsoid_miss_count = ellipsoid_miss_count_prior;

    std::vector<int> ellipsoid_intersected_count(ellipsoid_models.size(), 0);
    std::vector<int> pt_intersected_flag;

    std::cout << "processing section pts..." << std::endl;
    for(size_t i = 0; i < section_pt_ids_to_process.size()
	    ; ++i)
    {
    	int id = section_pt_ids_to_process[i];
	
    	double t = section.m_pt_timestamps[id];
    	std::vector<double> this_pt = section.m_pts[id];
    	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
    	std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, laser_calib_params);
    	std::vector<double> ray_dirn;
	
    	double meas_dist;
    	std::tie(ray_dirn, meas_dist) = calcRayDirn(ray_origin, this_pt);
	
    	std::vector<int> intersection_flag;
	std::vector<double> dist_along_ray;
	std::tie(intersection_flag, dist_along_ray) = sim.calcEllipsoidIntersections(
    	    ray_origin, ray_dirn);

    	if (!anyNonzeros(intersection_flag))
	{
    	    continue; 	// no intersections
	    pt_intersected_flag.push_back(0);
	}
	else
	    pt_intersected_flag.push_back(1);

	std::vector<int> sorted_intersecting_ids;
    	std::vector<double> sorted_dist_along_ray;
    	std::tie(sorted_intersecting_ids, sorted_dist_along_ray) =
    	    sortIntersectionFlag(intersection_flag, dist_along_ray);
    	std::vector<double> maha_dists_to_ellipsoids = 
    	    sim.calcMahaDistPtToEllipsoids(sorted_intersecting_ids, this_pt);

    	// log ellipsoid intersections
	for(auto j : sorted_intersecting_ids)
	    ellipsoid_intersected_count[j] += 1;
	
    	int ellipsoid_hit_id;
    	std::vector<int> ellipsoid_miss_ids;
    	std::tie(ellipsoid_hit_id, ellipsoid_miss_ids) = 
    	    sim.assignEllipsoidHitCredits(maha_dists_to_ellipsoids, sorted_intersecting_ids, 
					  sorted_dist_along_ray, meas_dist);

	// assign credits
	if (ellipsoid_hit_id != -1)
	    ellipsoid_hit_count[ellipsoid_hit_id] += 1;

	if (!ellipsoid_miss_ids.empty())
	    for(size_t j = 0; j < ellipsoid_miss_ids.size(); ++j)
		ellipsoid_miss_count[ellipsoid_miss_ids[j]] += 1;

	// debug
	// std::cout << "section pt id: " << id << std::endl;
	// std::cout << "t: " << t << std::endl;
	// std::cout << "this pt: " << std::endl;
	// dispVec(this_pt);
	// std::cout << "imu_pose " << std::endl;
	// dispVec(imu_pose);
	// std::cout << "ray origin " << std::endl;
	// dispVec(ray_origin);
	// std::cout << "ray dirn " << std::endl;
	// dispVec(ray_dirn);
	// std::cout << "meas dist " << meas_dist << std::endl;
	// std::cout << "sorted intersecting ids " << std::endl;
	// dispVec(sorted_intersecting_ids);
	// std::cout << "sorted dist along ray " << std::endl;
	// dispVec(sorted_dist_along_ray);
	// std::cout << "maha dists to ellipsoids " << std::endl;
	// dispVec(maha_dists_to_ellipsoids);
	// std::cout << "ellipsoid hit id: " << ellipsoid_hit_id << std::endl;
	// std::cout << "ellipsoid miss ids: "  << std::endl;
	// dispVec(ellipsoid_miss_ids);
    }

    // calc hit prob and add to models
    std::vector<double> hit_prob_vec(ellipsoid_models.size(), 1);
    for(size_t i = 0; i < hit_prob_vec.size(); ++i)
    {
	hit_prob_vec[i] = (double)(ellipsoid_hit_count[i]/(double)(ellipsoid_hit_count[i] + ellipsoid_miss_count[i]));
	ellipsoid_models[i].hit_prob = hit_prob_vec[i];
    }
    
    // stats
    std::cout << "fracs pts intersected: " << std::accumulate(pt_intersected_flag.begin(), pt_intersected_flag.end(), 0.0)/(double)section_pt_ids_to_process.size() << std::endl;
    std::vector<int> ellipsoid_missed_flag = negateLogicalVec(ellipsoid_intersected_count);
    std::cout << "fracs ellipsoids missed: " << std::accumulate(std::begin(ellipsoid_missed_flag), std::end(ellipsoid_missed_flag), 0.0)/(double)ellipsoid_missed_flag.size() << std::endl;

    // write to file
    std::cout << "writing out..." << std::endl;
    std::string rel_path_output = "data/ellipsoid_models_updated_hit_prob.txt";
    writeEllipsoidModelsToFile(ellipsoid_models, rel_path_output);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

