#include <algorithm>
#include <random>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Object.h>

#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>

using namespace lidar_sim;

TriangleModelSim::TriangleModelSim() :
    m_range_var(0.07),
    m_max_residual_for_hit(0.75),
    m_normal_dist(0, m_range_var),
    m_deterministic_sim(false)
{    
    std::random_device rd;
    std::mt19937 gen(rd());
    m_gen = gen;
}

void TriangleModelSim::loadTriangleModels(const std::string &rel_path_input)
{
    m_triangle_models = loadTriangleModelsFromFile(rel_path_input);
    fillCgalData();
}

void TriangleModelSim::setTriangleModels(const TriangleModels &triangle_models)
{
    m_triangle_models = triangle_models;
    fillCgalData();
}

void TriangleModelSim::fillCgalData()
{
    for(size_t i = 0; i < m_triangle_models.m_fit_pts.size(); ++i)
    {
	Point_3_cgal pt(m_triangle_models.m_fit_pts[i][0], m_triangle_models.m_fit_pts[i][1], m_triangle_models.m_fit_pts[i][2]);
	m_fit_pts_cgal.push_back(pt);
    }

    for(size_t i = 0; i < m_triangle_models.m_triangles.size(); ++i)
    {
	Triangle_3_cgal t(m_fit_pts_cgal[m_triangle_models.m_triangles[i][0]],
			  m_fit_pts_cgal[m_triangle_models.m_triangles[i][1]],
			  m_fit_pts_cgal[m_triangle_models.m_triangles[i][2]]);
	m_triangles_cgal.push_back(t);
    }
}

void TriangleModelSim::setLaserCalibParams(LaserCalibParams laser_calib_params)
{
    m_laser_calib_params = laser_calib_params;
}

std::tuple<std::vector<int>,
	   std::vector<double> > 
TriangleModelSim::calcTriIntersections(std::vector<double> ray_origin, std::vector<double> ray_dirn)
{
    if (m_triangles_cgal.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "TriangleModelSim: m_triangles_cgal is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }
	
    std::vector<int> intersection_flag(m_triangle_models.m_triangles.size(), 0);
    std::vector<double> dist_along_ray(m_triangle_models.m_triangles.size(), 0);
    
    Point_3_cgal ray_origin_cgal(ray_origin[0], ray_origin[1], ray_origin[2]);
    Direction_3_cgal ray_dirn_cgal(ray_dirn[0], ray_dirn[1], ray_dirn[2]);
    Ray_3_cgal ray_cgal(ray_origin_cgal, ray_dirn_cgal);
    
    Point_3_cgal pt_intersection;
    for(size_t i = 0; i < m_triangle_models.m_triangles.size(); ++i)
    {
	CGAL::Object obj_intersection;
	try
	{
	    obj_intersection = intersection(ray_cgal, m_triangles_cgal[i]);
	}
	catch (const std::exception &e)
	{
	    // std::cout << e.what() << std::endl;

	    // todo: deal with this better
	    // problem when meshes have repeated points as vertices
	    continue;
	}

	if(assign(pt_intersection, obj_intersection))
	{
	    intersection_flag[i] = 1;
	    double dist = 0;
	    for(size_t j = 0; j < 3; ++j)
		dist += std::pow(pt_intersection[j] - ray_origin[j], 2);
	    dist_along_ray[i] = std::sqrt(dist);
	}
    }
    
    return std::make_tuple(intersection_flag, dist_along_ray);
}

std::tuple<std::vector<std::vector<int> >,
	   std::vector<std::vector<double> > > 
TriangleModelSim::calcTriIntersections(std::vector<double> ray_origin, std::vector<std::vector<double> > ray_dirns)
{
    std::vector<std::vector<int> > intersection_flag;
    std::vector<std::vector<double> > dist_along_ray;
    
    for(size_t i = 0; i < ray_dirns.size(); ++i)
    {
	std::vector<int> flag;
	std::vector<double> dists;
	std::tie(flag, dists) = calcTriIntersections(ray_origin, ray_dirns[i]);
	intersection_flag.push_back(flag);
	dist_along_ray.push_back(dists);
    }

    return std::make_tuple(intersection_flag, dist_along_ray);
}

std::tuple<int, std::vector<int>, double>
TriangleModelSim::assignTriHitCredits(std::vector<double> sorted_dists_to_tri, 
				      std::vector<int> sorted_intersecting_ids, double measured_range)
{
    std::vector<double> residual_ranges(sorted_dists_to_tri.size(), 0);
    std::vector<double> abs_residual_ranges(sorted_dists_to_tri.size(), 0);
    for(size_t i = 0; i < sorted_dists_to_tri.size(); ++i)
    {
	residual_ranges[i] = measured_range - sorted_dists_to_tri[i];
	abs_residual_ranges[i] = std::abs(residual_ranges[i]);
    }

    auto min_it = std::min_element(std::begin(abs_residual_ranges), std::end(abs_residual_ranges));
    int min_posn = std::distance(std::begin(abs_residual_ranges), min_it);
    double min_residual_range = residual_ranges[min_posn];
    
    std::vector<int> tri_miss_ids;
    for(size_t i = 0; i < (size_t)min_posn; ++i)
	tri_miss_ids.push_back(sorted_intersecting_ids[i]);

    int tri_hit_id;
    if(min_residual_range < m_max_residual_for_hit)
	tri_hit_id = sorted_intersecting_ids[min_posn];
    else
    {
	tri_hit_id = -1;
	tri_miss_ids.push_back(sorted_intersecting_ids[min_posn]);
    }

    return std::make_tuple(tri_hit_id, tri_miss_ids, min_residual_range);
}

double TriangleModelSim::getMaxResidualForHit()
{
    return m_max_residual_for_hit;
}

void TriangleModelSim::writeTrianglesToFile(std::string rel_path_output)
{
    std::ofstream file(rel_path_output);
    std::cout << "Writing triangles to: " << rel_path_output << std::endl;

    file << "pts" << std::endl;
    
    for(size_t i = 0; i < m_triangle_models.m_fit_pts.size(); ++i)
	file << m_triangle_models.m_fit_pts[i][0] << " " <<
	    m_triangle_models.m_fit_pts[i][1] << " " <<
	    m_triangle_models.m_fit_pts[i][2] << std::endl;

    file << "triangles" << std::endl;
    for(size_t i = 0; i < m_triangle_models.m_triangles.size(); ++i)
	file << m_triangle_models.m_triangles[i][0] << " " <<
	    m_triangle_models.m_triangles[i][1] << " " <<
	    m_triangle_models.m_triangles[i][2] << " " << m_triangle_models.m_hit_prob_vec[i] << std::endl;
    
    file.close();
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> >
TriangleModelSim::simPtsGivenIntersections(std::vector<double> ray_origin, std::vector<std::vector<double> > ray_dirns,
					   std::vector<std::vector<int> > intersection_flag, std::vector<std::vector<double> > dist_along_ray)
{
    size_t n_rays = intersection_flag.size();

    Pts sim_pts(n_rays, std::vector<double>(3, 0));
    std::vector<int> hit_flag(n_rays, 0);

    for(size_t i = 0; i < n_rays; ++i)
    {
	if (!anyNonzeros(intersection_flag[i]))
	{
	    hit_flag[i] = 0;
	    continue;
	}
	else
	    hit_flag[i] = 1;

	std::vector<int> sorted_intersecting_ids;
	std::vector<double> sorted_dist_along_ray_intersections;
	std::tie(sorted_intersecting_ids, sorted_dist_along_ray_intersections) =
	    sortIntersectionFlag(intersection_flag[i], dist_along_ray[i]);

	std::vector<double> hit_prob_vec(sorted_intersecting_ids.size());
	for(size_t j = 0; j < sorted_intersecting_ids.size(); ++j)
	    hit_prob_vec[j] = m_triangle_models.m_hit_prob_vec[sorted_intersecting_ids[j]];

	int hit_ellipsoid_id;
	bool hit_bool;
	std::tie(hit_ellipsoid_id, hit_bool) =
	    sampleHitId(hit_prob_vec, sorted_intersecting_ids, m_deterministic_sim);

	if (!hit_bool)
	{
	    hit_flag[i] = 0;
	    continue;
	}

	double noise;
	if (m_deterministic_sim)
	    noise = 0;
	else
	    noise = m_normal_dist(m_gen);

	double sim_range = dist_along_ray[i][hit_ellipsoid_id] + noise;
	 
	for(size_t j = 0; j < 3; ++j)
	    sim_pts[i][j] = ray_origin[j] + sim_range*ray_dirns[i][j];
    }

    return std::make_tuple(sim_pts, hit_flag);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> >
TriangleModelSim::simPtsGivenPose(const std::vector<double> &imu_pose)
{
    // rays
    std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, m_laser_calib_params);
    std::vector<std::vector<double> > ray_dirns = genRayDirnsWorldFrame(imu_pose, m_laser_calib_params);

    return simPtsGivenRays(ray_origin, ray_dirns);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
TriangleModelSim::simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses)
{
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    for(size_t i = 0; i < imu_poses.size(); ++i)
    {
	std::vector<std::vector<double> > this_sim_pts;
	std::vector<int> this_hit_flag;
	std::tie(this_sim_pts, this_hit_flag) = simPtsGivenPose(imu_poses[i]); 
	for(size_t j = 0; j < this_sim_pts.size(); ++j)
	{
	    sim_pts.push_back(this_sim_pts[j]);
	    hit_flag.push_back(this_hit_flag[j]);
	}
    }

    return std::make_tuple(sim_pts, hit_flag);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> >
TriangleModelSim::simPtsGivenRays(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns)
{
    // intersections
    std::vector<std::vector<int> > intersection_flag;
    std::vector<std::vector<double> > dist_along_ray;
    std::tie(intersection_flag, dist_along_ray) = calcTriIntersections(ray_origin, ray_dirns);

    // sim
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    std::tie(sim_pts, hit_flag) = simPtsGivenIntersections(ray_origin, ray_dirns,
							   intersection_flag, dist_along_ray);

    applyMaxRangeFilter(ray_origin, sim_pts, hit_flag, 
			m_laser_calib_params.intrinsics.max_range);

    // debug
    // std::cout << "TriangleModelSim: anyNonzeros(intersection_flag): " 
    // 	      << anyNonzeros(intersection_flag[0]) << std::endl;	

    return std::make_tuple(sim_pts, hit_flag);
}

std::tuple<std::vector<double>, int >
TriangleModelSim::simPtGivenRay(const std::vector<double> &ray_origin, const std::vector<double> &ray_dirn)
{
    auto ray_dirns = wrapDataInVec(ray_dirn);
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hits_flag;
    std::tie(sim_pts, hits_flag) = simPtsGivenRays(ray_origin, ray_dirns);
    std::vector<double> sim_pt = sim_pts[0];
    int hit_flag = hits_flag[0];

    return std::make_tuple(sim_pt, hit_flag);
}

void TriangleModelSim::setDeterministicSim(const bool choice)
{
    m_deterministic_sim = choice;
}
