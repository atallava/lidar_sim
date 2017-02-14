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
    class SectionModelSim {
    public:
	SectionModelSim();;
	void loadEllipsoidModelBlocks(const std::vector<std::string> &rel_path_model_blocks);
	void loadTriangleModelBlocks(const std::vector<std::string> &rel_path_model_blocks);
	void loadBlockInfo(const std::string rel_path_imu_posn_nodes, const std::string rel_path_block_node_ids_ground, const std::string rel_path_block_node_ids_non_ground);
	std::vector<int> getPoseEllipsoidBlockMembership(const std::vector<double> &imu_pose);
	std::vector<int> getPoseTriangleBlockMembership(const std::vector<double> &imu_pose);
	std::vector<int> getPoseBlockMembership(const std::vector<double> &imu_pose, const std::vector<std::vector<int> > &block_node_ids);
	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenPose(const std::vector<double> &imu_pose);
	std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
	    simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses);

	std::vector<EllipsoidModelSim> m_ellipsoid_model_sims;
	std::vector<TriangleModelSim> m_triangle_model_sims;
	std::vector<std::vector<double> > m_imu_posn_nodes;
	std::vector<std::vector<int> > m_block_node_ids_ground;
	std::vector<std::vector<int> > m_block_node_ids_non_ground;

    private:
	LaserCalibParams m_laser_calib_params;
	double m_max_dist_to_node_for_membership;
    };
}