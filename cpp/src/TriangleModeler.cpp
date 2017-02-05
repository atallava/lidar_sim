#include <algorithm>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "interpolation.h"

#include <lidar_sim/TriangleModeler.h>
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

TriangleModeler::TriangleModeler() :
    m_debug_flag(0),
    m_rbf_radius(0.1),
    m_rbf_layers(1),
    m_rbf_reg(1e-3),
    m_fit_pts_padding(5),
    m_fit_pts_node_resn(1),
    m_max_dist_to_projn(1.5),
    m_default_hit_prob(1),
    m_hit_count_prior(1),
    m_miss_count_prior(0)
{    
}

void TriangleModeler::createTriangleModels(const std::string rel_path_pts)
{
    if (m_debug_flag)
	std::cout << "TriangleModeler: creating triangle models..." << std::endl;

    loadPts(rel_path_pts);
    fitSmoothedPts();
    delaunayTriangulate();
    calcTrianglesFromTriangulation();
    std::vector<double> default_hit_prob_vec(m_triangles.size(), m_default_hit_prob);
    m_hit_prob_vec = default_hit_prob_vec;
}

void TriangleModeler::loadPts(const std::string rel_path_pts)
{
    m_pts = loadPtsFromXYZFile(rel_path_pts);
}

void TriangleModeler::fitSmoothedSurface()
{
    if (m_debug_flag)
	std::cout << "TriangleModeler: fitting smoothed surface..." << std::endl;

    alglib::rbfreport rep;
    alglib::rbfcreate(2, 1, m_surface_model); // 2d input, 1d output
    alglib::real_2d_array pts_alglib = convertStlPtsToAlglibPts(m_pts);
    alglib::rbfsetpoints(m_surface_model, pts_alglib);
    alglib::rbfsetalgomultilayer(m_surface_model, m_rbf_radius, m_rbf_layers, m_rbf_reg);
    alglib::rbfbuildmodel(m_surface_model, rep);
}


void TriangleModeler::fitSmoothedPts()
{
    if (m_debug_flag)
	std::cout << "TriangleModeler: fitting smoothed points..." << std::endl;

    fitSmoothedSurface();

    std::vector<double> x_node_vec;
    std::vector<double> y_node_vec;
    std::tie(x_node_vec, y_node_vec) = genNodeVecsForSmoothedFit(m_pts);
    std::vector<std::vector<double> > xy_nodes(x_node_vec.size()*y_node_vec.size(), std::vector<double>(2));
    for(size_t i = 0; i < x_node_vec.size(); ++i)
	for(size_t j = 0; j < y_node_vec.size(); ++j)
	{
	    xy_nodes[i*y_node_vec.size() + j][0] = x_node_vec[i];
	    xy_nodes[i*y_node_vec.size() + j][1] = y_node_vec[j];
	}

    std::vector<std::vector<double> > xy_fit = cutNodesToPtsProjn(xy_nodes);
    
    // z for the fit pts
    for(size_t i = 0 ; i < xy_fit.size(); ++i)
    {
	std::vector<double> vec = {xy_fit[i][0], xy_fit[i][1], 
				   alglib::rbfcalc2(m_surface_model, xy_fit[i][0], xy_fit[i][1])};
	m_fit_pts.push_back(vec);
    }
}

std::vector<std::vector<double> >
TriangleModeler::cutNodesToPtsProjn(std::vector<std::vector<double> > xy_nodes)
{
    std::vector<std::vector<double> > xy_pts(m_pts.size(), std::vector<double>(2,0));
    for(size_t i = 0; i < m_pts.size(); ++i)
    {
	xy_pts[i][0] = m_pts[i][0]; 
	xy_pts[i][1] = m_pts[i][1];
    }
    
    std::vector<int> nn_ids;
    std::tie(nn_ids, std::ignore) = nearestNeighbors(xy_pts, xy_nodes);
    std::vector<std::vector<double> > xy_fit;

    for(size_t i = 0; i < xy_nodes.size(); ++i)
    {
	double min_dist = euclideanDist(xy_nodes[i], xy_pts[nn_ids[i]]);
	
	if (min_dist < m_max_dist_to_projn)
	    xy_fit.push_back(xy_nodes[i]);
    }

    return xy_fit;
}

std::tuple<std::vector<double>, std::vector<double> >
TriangleModeler::genNodeVecsForSmoothedFit(const std::vector<std::vector<double> > &pts)
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    std::tie(x, y, z) = getVecsFromPts(pts);

    auto x_min_it = std::min_element(std::begin(x), std::end(x));
    double x_min = *x_min_it;
    auto x_max_it = std::max_element(std::begin(x), std::end(x));
    double x_max = *x_max_it;
    auto y_min_it = std::min_element(std::begin(y), std::end(y));
    double y_min = *y_min_it;
    auto y_max_it = std::max_element(std::begin(y), std::end(y));
    double y_max = *y_max_it;

    std::vector<double> x_node_vec;
    std::vector<double> y_node_vec;
    double val = x_min - m_fit_pts_padding;
    while (val < (x_max + m_fit_pts_padding))
    {
	x_node_vec.push_back(val);
	val += m_fit_pts_node_resn;
    }

    val = y_min - m_fit_pts_padding;
    while (val < (y_max + m_fit_pts_padding))
    {
	y_node_vec.push_back(val);
	val += m_fit_pts_node_resn;
    }

    return std::make_tuple(x_node_vec, y_node_vec);
}

void TriangleModeler::delaunayTriangulate()
{
    if (m_debug_flag)
	std::cout << "TriangleModeler: delaunay triangulation..." << std::endl;

    std::vector< std::pair<Point_cgal, unsigned> > points_cgal;
    for(size_t i = 0; i < m_fit_pts.size(); ++i)
	points_cgal.push_back(
	    std::make_pair(Point_cgal(m_fit_pts[i][0], m_fit_pts[i][1]), i));
    
    m_triangulation.insert(points_cgal.begin(), points_cgal.end());
}

void TriangleModeler::calcTrianglesFromTriangulation()
{
    if (m_debug_flag)
	std::cout << "TriangleModeler: triangles from triangulation..." << std::endl;

    for(Delaunay_cgal::Finite_faces_iterator fit = m_triangulation.finite_faces_begin();
	fit != m_triangulation.finite_faces_end(); ++fit) 
    {
	Delaunay_cgal::Face_handle face = fit;
	std::vector<int> triangle;
	for(size_t i = 0; i < 3; ++i)
	    triangle.push_back(face->vertex(i)->info());
	m_triangles.push_back(triangle);
    }
}

void TriangleModeler::writeTrianglesToFile(std::string rel_path_output)
{
    std::ofstream file(rel_path_output);
    std::cout << "TriangleModeler: writing triangles to: " << rel_path_output << std::endl;

    file << "pts" << std::endl;
    
    for(size_t i = 0; i < m_fit_pts.size(); ++i)
	file << m_fit_pts[i][0] << " " <<
	    m_fit_pts[i][1] << " " <<
	    m_fit_pts[i][2] << std::endl;

    file << "triangles" << std::endl;
    for(size_t i = 0; i < m_triangles.size(); ++i)
	file << m_triangles[i][0] << " " <<
	    m_triangles[i][1] << " " <<
	    m_triangles[i][2] << " " << m_hit_prob_vec[i] << std::endl;
    
    file.close();
}

void TriangleModeler::setDebugFlag(int flag)
{
    m_debug_flag = flag;
}

void TriangleModeler::calcHitProb(std::string rel_path_section, PoseServer imu_pose_server)
{
    // sim object
    TriangleModelSim sim;
    const LaserCalibParams laser_calib_params;
    sim.setLaserCalibParams(laser_calib_params);
    sim.m_triangles = m_triangles;
    sim.m_fit_pts = m_fit_pts;
    sim.m_hit_prob_vec = m_hit_prob_vec;

    // section
    SectionLoader section(rel_path_section);
    
    // section ids to process
    std::vector<int> section_pt_ids_to_process;
    std::tie(section_pt_ids_to_process, std::ignore) = nearestNeighbors(section.m_pts, m_pts);

    size_t n_tris = sim.m_triangles.size();
    std::vector<int> tri_hit_count_prior(n_tris, m_hit_count_prior);
    std::vector<int> tri_miss_count_prior(n_tris, m_miss_count_prior);

    std::vector<int> tri_hit_count = tri_hit_count_prior;
    std::vector<int> tri_miss_count = tri_miss_count_prior;

    std::vector<double> residual_ranges;
    std::vector<double> filtered_residual_ranges;

    for(size_t i = 0; i < section_pt_ids_to_process.size(); ++i)
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
    	    continue; 	// no hits

    	// intersections
	std::vector<int> sorted_intersecting_ids;
    	std::vector<double> sorted_dists_to_tri;
    	std::tie(sorted_intersecting_ids, sorted_dists_to_tri) =
    	    sortIntersectionFlag(intersection_flag, dists_to_tri);
	
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
	if (this_residual_range < sim.getMaxResidualForHit())
	    filtered_residual_ranges.push_back(this_residual_range);
    }

    if (m_debug_flag)
	std::cout << "TriangleModeler: n rays intersected: " << residual_ranges.size() << std::endl;

    // calculate hit prob
    for(size_t i = 0; i < m_triangles.size(); ++i)
	m_hit_prob_vec[i] = (double)(tri_hit_count[i]/(double)(tri_hit_count[i] + tri_miss_count[i]));

    double range_var = calcVariance(filtered_residual_ranges);
    std::cout << "range variance: " << range_var << std::endl;
}