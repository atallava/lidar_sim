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
#include <lidar_sim/ModelingUtils.h>

namespace lidar_sim {
    class SectionModelSim {
    public:
	SectionModelSim();;
	void loadEllipsoidModelBlocks(const std::vector<std::string> &rel_path_model_blocks);
	void loadTriangleModelBlocks(const std::vector<std::string> &rel_path_model_blocks);
	void loadBlockInfo(const std::string rel_path_imu_posn_nodes, const std::string rel_path_block_node_ids_ground, const std::string rel_path_block_node_ids_non_ground);

	std::vector<int> getPosnEllipsoidBlockMembership(const std::vector<double> &imu_pose);
	std::vector<int> getPosnTriangleBlockMembership(const std::vector<double> &imu_pose);
	std::vector<int> getPoseBlockMembership(const std::vector<double> &imu_pose, const std::vector<std::vector<int> > &block_node_ids);
	std::vector<int> getPosnBlockMembership(const std::vector<double> &posn, const std::vector<std::vector<int> > &block_node_ids);
	
	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenPose(const std::vector<double> &imu_pose);
	std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
	    simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses);
	std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
	    simPtsGivenRays(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns);

	void setDeterministicSim(const bool choice);

	std::vector<EllipsoidModelSim> m_ellipsoid_model_sims;
	std::vector<TriangleModelSim> m_triangle_model_sims;
	std::vector<EllipsoidModel> m_ellipsoid_models_all;
	std::vector<std::vector<double> > m_imu_posn_nodes;
	std::vector<std::vector<int> > m_block_node_ids_ground;
	std::vector<std::vector<int> > m_block_node_ids_non_ground;
	LaserCalibParams m_laser_calib_params;

    private:
	double m_max_dist_to_node_for_membership;
	bool m_deterministic_sim;
    };
}
