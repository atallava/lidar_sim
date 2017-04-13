#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

#include <lidar_sim/SectionModelSim.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    class NearestNeighborSim {
    public:
	NearestNeighborSim();;
	void loadTrainPts(const std::string rel_path_pts);
	void setDebugFlag(const int value);
	void subsamplePts();

	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenPose(const std::vector<double> &imu_pose);
	std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
	    simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses);
	std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
	    simPtsGivenRays(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns);

	std::vector<std::vector<double> > m_pts;
	LaserCalibParams m_laser_calib_params;

    private:
	int m_debug_flag;
	double m_max_perp_dist_for_hit;
	double m_range_var;
	int m_max_pts;
    };
}
