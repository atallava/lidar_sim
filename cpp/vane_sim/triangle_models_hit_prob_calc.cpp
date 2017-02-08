#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/TriangleModelSim.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    std::cout << "loading data..." << std::endl;
    // load models into sim
    std::string rel_path_triangle_models = "data/triangle_models.txt";
    TriangleModelSim sim;
    const LaserCalibParams laser_calib_params;
    sim.loadTriangleModels(rel_path_triangle_models);
    sim.setLaserCalibParams(laser_calib_params);

    // load section
    std::string rel_path_section = "data/section_03_world_frame_subsampled_timed.xyz";
    SectionLoader section(rel_path_section);

    // pts from xyz
    std::string rel_path_xyz = "data/rim_stretch_ground_train.asc";
    std::vector<std::vector<double> > train_pts = loadPtsFromXYZFile(rel_path_xyz);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    std::cout << "calculating section ids to process..." << std::endl;
    // section ids to process
    std::vector<int> section_pt_ids_to_process;
    std::tie(section_pt_ids_to_process, std::ignore) = nearestNeighbors(section.m_pts, train_pts);

    size_t n_tris = sim.m_triangles.size();
    std::vector<int> tri_hit_count_prior(n_tris, 1);
    std::vector<int> tri_miss_count_prior(n_tris, 0);

    std::vector<int> tri_hit_count = tri_hit_count_prior;
    std::vector<int> tri_miss_count = tri_miss_count_prior;

    std::vector<double> residual_ranges;
    std::vector<double> filtered_residual_ranges;

    std::vector<int> tri_intersected_count(sim.m_triangles.size(), 0);
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
    	std::vector<double> dists_to_tri;
    	std::tie(intersection_flag, dists_to_tri) = sim.calcTriIntersections(
    	    ray_origin, ray_dirn);

    	if (!anyNonzeros(intersection_flag))
	{
    	    continue; 	// no intersections
	    pt_intersected_flag.push_back(0);
	}
	else
	    pt_intersected_flag.push_back(1);

    	// intersections
	std::vector<int> sorted_intersecting_ids;
    	std::vector<double> sorted_dists_to_tri;
    	std::tie(sorted_intersecting_ids, sorted_dists_to_tri) =
    	    sortIntersectionFlag(intersection_flag, dists_to_tri);
	
    	// log ellipsoid intersections
	for(auto j : sorted_intersecting_ids)
	    tri_intersected_count[j] += 1;
	
	// hit miss credits
	int tri_hit_id;
	std::vector<int> tri_miss_ids;
	double this_residual_range;
	std::tie(tri_hit_id, tri_miss_ids, this_residual_range) = 
	    sim.assignTriHitCredits(sorted_dists_to_tri, sorted_intersecting_ids, meas_dist);

	if (tri_hit_id >= 0)
	    tri_hit_count[tri_hit_id] += 1;

	if (!tri_miss_ids.empty())
	    for(size_t j = 0; j < tri_miss_ids.size(); ++j)
		tri_miss_count[tri_miss_ids[j]] += 1;

	residual_ranges.push_back(this_residual_range);
	if (std::abs(this_residual_range) < sim.getMaxResidualForHit())
	    filtered_residual_ranges.push_back(this_residual_range);

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
	// std::cout << "residual range " << this_residual_range << std::endl;
	// std::cout << "sorted intersecting ids " << std::endl;
	// dispVec(sorted_intersecting_ids);
	// std::cout << "sorted dists to tri " << std::endl;
	// dispVec(sorted_dists_to_tri);
	// std::cout << "tri hit id: " << tri_hit_id << std::endl;
	// std::cout << "tri miss ids: "  << std::endl;
	// dispVec(tri_miss_ids);
    }

    std::cout << "n rays intersected: " << residual_ranges.size() << std::endl;

    // calculate hit prob
    std::vector<double> hit_prob_vec(n_tris, 1);
    for(size_t i = 0; i < hit_prob_vec.size(); ++i)
	hit_prob_vec[i] = (double)(tri_hit_count[i]/(double)(tri_hit_count[i] + tri_miss_count[i]));

    sim.m_hit_prob_vec = hit_prob_vec;

    double range_var = calcVariance(filtered_residual_ranges);
    std::cout << "range variance: " << range_var << std::endl;

    // stats
    std::cout << "n pts: " << section_pt_ids_to_process.size() << std::endl;
    std::cout << "fracs pts intersected: " << std::accumulate(pt_intersected_flag.begin(), pt_intersected_flag.end(), 0.0)/(double)section_pt_ids_to_process.size() << std::endl;
    std::vector<int> tri_missed_flag = negateLogicalVec(tri_intersected_count);
    std::cout << "fracs tri missed: " << std::accumulate(std::begin(tri_missed_flag), std::end(tri_missed_flag), 0.0)/(double)tri_missed_flag.size() << std::endl;

    // write out
    std::string rel_path_triangles = "data/triangle_models_updated_hit_prob.txt";
    sim.writeTrianglesToFile(rel_path_triangles);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

