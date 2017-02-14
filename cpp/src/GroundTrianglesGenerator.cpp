#include <algorithm>
#include <random>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/GroundTrianglesGenerator.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

GroundTrianglesGenerator::GroundTrianglesGenerator() :
    m_debug_flag(0),
    m_avg_ground_depth_from_imu(6),
    m_lateral_dist(15),
    m_fit_pts_node_resn(1),
    m_max_triangle_side(15),
    m_default_hit_prob(1)
{    
}

void GroundTrianglesGenerator::createTriangleModels(const std::string rel_path_imu_posns)
{
    std::vector<std::vector<double> > imu_posns = loadArray(rel_path_imu_posns, 3);
    createTriangleModels(imu_posns);
}

void GroundTrianglesGenerator::createTriangleModels(const std::vector<std::vector<double> > &imu_posns)
{
    if (m_debug_flag)
	std::cout << "GroundTriangleGenerator: creating triangle models..." << std::endl;

    fitPts(imu_posns);
    delaunayTriangulate();
    calcTrianglesFromTriangulation();
    m_hit_prob_vec = std::vector<double>(m_triangles.size(), m_default_hit_prob);
    filterTriangles();
}

void GroundTrianglesGenerator::fitPts(const std::vector<std::vector<double> > &imu_posns)
{
    if (m_debug_flag)
	std::cout << "GroundTriangleGenerator: fitting pts..." << std::endl;

    int steps = 0.5*m_lateral_dist/m_fit_pts_node_resn;
    m_fit_pts.clear();

    for(size_t i = 0; i < imu_posns.size(); ++i)
    {
	// step dirn is roughly perpendicular to current xy tangent
	std::vector<double> xy_tangent(2, 0);
	if (i < (imu_posns.size()-1))
	{
	    xy_tangent[0] = imu_posns[i+1][0] - imu_posns[i][0];
	    xy_tangent[1] = imu_posns[i+1][1] - imu_posns[i][1];
	}
	else
	{
	    xy_tangent[0] = imu_posns[i][0] - imu_posns[i-1][0];
	    xy_tangent[1] = imu_posns[i][1] - imu_posns[i-1][1];
	}
	std::vector<double> step_dirn = getPerpUnitVec2(xy_tangent);

	for(int j = -steps; j <= steps; ++j)
	{
	    std::vector<double> fit_pt(3, 0);
	    fit_pt[0] = imu_posns[i][0] + j*step_dirn[0];
	    fit_pt[1] = imu_posns[i][1] + j*step_dirn[1];
	    fit_pt[2] = imu_posns[i][2] - m_avg_ground_depth_from_imu;

	    m_fit_pts.push_back(fit_pt);
	}
    }

}

void GroundTrianglesGenerator::delaunayTriangulate()
{
    if (m_debug_flag)
	std::cout << "GroundTrianglesGenerator: delaunay triangulation..." << std::endl;

    if (m_fit_pts.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "GroundTrianglesGenerator: m_fit_pts is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    std::vector< std::pair<Point_cgal, unsigned> > points_cgal;
    for(size_t i = 0; i < m_fit_pts.size(); ++i)
	points_cgal.push_back(
	    std::make_pair(Point_cgal(m_fit_pts[i][0], m_fit_pts[i][1]), i));
    
    m_triangulation.insert(points_cgal.begin(), points_cgal.end());
}

void GroundTrianglesGenerator::calcTrianglesFromTriangulation()
{
    if (m_debug_flag)
	std::cout << "GroundTrianglesGenerator: triangles from triangulation..." << std::endl;

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

void GroundTrianglesGenerator::writeTrianglesToFile(std::string rel_path_output)
{
    if (m_fit_pts.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "GroundTrianglesGenerator: m_fit_pts is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    if (m_triangles.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "GroundTrianglesGenerator: m_triangles is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    std::ofstream file(rel_path_output);
    std::cout << "GroundTrianglesGenerator: writing triangles to: " << rel_path_output << std::endl;

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

void GroundTrianglesGenerator::writeTrianglesFitPtsToFile(std::string rel_path_output)
{
    std::ofstream file(rel_path_output);
    std::cout << "GroundTrianglesGenerator: writing triangles fit pts to: " << rel_path_output << std::endl;

    for(size_t i = 0; i < m_fit_pts.size(); ++i)
	file << m_fit_pts[i][0] << " " <<
	    m_fit_pts[i][1] << " " <<
	    m_fit_pts[i][2] << std::endl;

    file.close();
}

void GroundTrianglesGenerator::setDebugFlag(int flag)
{
    m_debug_flag = flag;
}

void GroundTrianglesGenerator::filterTriangles()
{
    if (m_debug_flag)
	std::cout << "GroundTrianglesGenerator: filtering triangles..." << std::endl;

    std::vector<int> flag(m_triangles.size(), 1);
    for(size_t i = 0; i < m_triangles.size(); ++i)
    {
	std::vector<double> v0 = m_fit_pts[m_triangles[i][0]];
	std::vector<double> v1 = m_fit_pts[m_triangles[i][1]];
	std::vector<double> v2 = m_fit_pts[m_triangles[i][2]];

	double s0 = euclideanDist(v0, v1);
	double s1 = euclideanDist(v1, v2);
	double s2 = euclideanDist(v2, v0);
	
	bool condn = (s0 > m_max_triangle_side) || 
	    (s1 > m_max_triangle_side) ||
	    (s2 > m_max_triangle_side);

	if (condn)
	    flag[i] = 0;
    }

    std::vector<std::vector<int> > filtered_triangles = 
	logicalSubsetArray(m_triangles, flag);
    if (m_debug_flag)
	std::cout << "GroundTrianglesGenerator: fracn triangles retained: " 
		  << filtered_triangles.size()/(double)m_triangles.size() << std::endl;

    m_triangles = filtered_triangles;
}
