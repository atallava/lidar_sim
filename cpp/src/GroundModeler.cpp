#include <algorithm>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "interpolation.h"

#include <lidar_sim/GroundModeler.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

GroundModeler::GroundModeler() :
    m_rbf_radius(0.1),
    m_rbf_layers(1),
    m_rbf_reg(1e-3),
    m_fit_pts_padding(5),
    m_fit_pts_node_resn(1),
    m_max_dist_to_projn(1.5),
    m_default_hit_prob(1)
{    
}

void GroundModeler::fitSmoothedSurface()
{
    alglib::rbfreport rep;
    alglib::rbfcreate(2, 1, m_surface_model); // 2d input, 1d output
    alglib::real_2d_array pts_alglib = convertStlPtsToAlglibPts(m_pts);
    alglib::rbfsetpoints(m_surface_model, pts_alglib);
    alglib::rbfsetalgomultilayer(m_surface_model, m_rbf_radius, m_rbf_layers, m_rbf_reg);
    alglib::rbfbuildmodel(m_surface_model, rep);
}


void GroundModeler::fitSmoothedPts()
{
    std::cout << "building surface model" << std::endl;
    fitSmoothedSurface();

    std::cout << "getting node vecs" << std::endl;
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

    std::cout << "cutting to projection" << std::endl;
    std::vector<std::vector<double> > xy_fit = cutNodesToPtsProjn(xy_nodes);
    
    std::cout << "calculating z for fit pts" << std::endl;
    // z for the fit pts
    for(size_t i = 0 ; i < xy_fit.size(); ++i)
    {
	std::vector<double> vec = {xy_fit[i][0], xy_fit[i][1], 
				   alglib::rbfcalc2(m_surface_model, xy_fit[i][0], xy_fit[i][1])};
	m_fit_pts.push_back(vec);
    }
}

std::vector<std::vector<double> >
GroundModeler::cutNodesToPtsProjn(std::vector<std::vector<double> > xy_nodes)
{
    
    std::vector<std::vector<double> > xy_pts(m_pts.size(), std::vector<double>(2,0));
    for(size_t i = 0; i < m_pts.size(); ++i)
    {
	xy_pts[i][0] = m_pts[i][0]; 
	xy_pts[i][1] = m_pts[i][1];
    }
    
    std::vector<int> nn_ids = nearestNeighbors(xy_pts, xy_nodes);
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
GroundModeler::genNodeVecsForSmoothedFit(const std::vector<std::vector<double> > &pts)
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

void GroundModeler::delaunayTriangulate()
{
    std::vector< std::pair<Point_cgal, unsigned> > points_cgal;
    for(size_t i = 0; i < m_fit_pts.size(); ++i)
	points_cgal.push_back(
	    std::make_pair(Point_cgal(m_fit_pts[i][0], m_fit_pts[i][1]), i));
    
    m_triangulation.insert(points_cgal.begin(), points_cgal.end());

    calcTrianglesFromTriangulation();
}

void GroundModeler::calcTrianglesFromTriangulation()
{
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

void GroundModeler::writeTrianglesToFile(std::string rel_path_output)
{
    std::ofstream file(rel_path_output);
    std::cout << "Writing triangles to: " << rel_path_output << std::endl;

    file << "pts" << std::endl;
    
    for(size_t i = 0; i < m_fit_pts.size(); ++i)
	file << m_fit_pts[i][0] << " " <<
	    m_fit_pts[i][1] << " " <<
	    m_fit_pts[i][2] << std::endl;

    file << "triangles" << std::endl;
    for(size_t i = 0; i < m_triangles.size(); ++i)
	file << m_triangles[i][0] << " " <<
	    m_triangles[i][1] << " " <<
	    m_triangles[i][2] << " " << m_default_hit_prob << std::endl;
    
    file.close();
}
