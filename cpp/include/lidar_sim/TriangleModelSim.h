#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/CgalTypedefs.h>
#include <lidar_sim/TriangleModels.h>

namespace lidar_sim {
    class TriangleModelSim {
    public:
	TriangleModelSim();
	void loadTriangleModels(const std::string &rel_path_models);
	void setTriangleModels(const TriangleModels &triangle_models);
	void setLaserCalibParams(LaserCalibParams laser_calib_params);
	void fillCgalData();
	double getMaxResidualForHit();
	
	std::tuple<std::vector<int>,
	    std::vector<double> > calcTriIntersections(std::vector<double> ray_origin, std::vector<double> ray_dirn);
	std::tuple<std::vector<std::vector<int> >,
	    std::vector<std::vector<double> > > calcTriIntersections(std::vector<double> ray_origin, std::vector<std::vector<double> > ray_dirns);
	std::tuple<int, std::vector<int>, double >
	    assignTriHitCredits(std::vector<double> sorted_dists_to_tri, 
				std::vector<int> sorted_intersecting_ids, double measured_range);
	void writeTrianglesToFile(std::string rel_path_output);
	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenIntersections(std::vector<double> ray_origin, std::vector<std::vector<double> > ray_dirns,
				     std::vector<std::vector<int> > intersection_flag, std::vector<std::vector<double> > dist_along_ray);
	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenPose(const std::vector<double> &imu_pose);

	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses);

	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenRays(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns);
	std::tuple<std::vector<double>, int >
	    simPtGivenRay(const std::vector<double> &ray_origin, const std::vector<double> &ray_dirn);
	void setDeterministicSim(const bool choice);

	TriangleModels m_triangle_models;
	double m_range_var;
	std::vector<Triangle_3_cgal> m_triangles_cgal;
	std::vector<Point_3_cgal> m_fit_pts_cgal;
	LaserCalibParams m_laser_calib_params;
	double m_max_residual_for_hit;

    private:
	std::mt19937 m_gen;
	std::normal_distribution<> m_normal_dist;
	bool m_deterministic_sim;
    };
}
