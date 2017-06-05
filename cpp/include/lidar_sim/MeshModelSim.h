#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>

namespace lidar_sim {
    class MeshModelSim {
    public:
	MeshModelSim();
	void loadObjectMeshes(const std::vector<std::string> &rel_path_object_meshes);
	void calcObjectCentroids();
	std::vector<int> calcObjectIdsForSim(const std::vector<double> &ray_origin, 
					     const std::vector<double> &ray_dirn);
	/* void loadTriangleModelBlocks(const std::vector<std::string> &rel_path_model_blocks); */
	/* void loadBlockInfo(const std::string rel_path_imu_posn_nodes, const std::string rel_path_block_node_ids_ground, const std::string rel_path_block_node_ids_non_ground); */

	/* std::vector<int> getPosnTriangleBlockMembership(const std::vector<double> &imu_pose); */
	/* std::vector<int> getPoseBlockMembership(const std::vector<double> &imu_pose, const std::vector<std::vector<int> > &block_node_ids); */
	/* std::vector<int> getPosnBlockMembership(const std::vector<double> &posn, const std::vector<std::vector<int> > &block_node_ids); */
	
	/* std::tuple<std::vector<std::vector<double> >, std::vector<int> > */
	/*     simPtsGivenPose(const std::vector<double> &imu_pose); */
	/* std::tuple<std::vector<std::vector<double> >, std::vector<int> >  */
	/*     simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses); */
	/* std::tuple<std::vector<std::vector<double> >, std::vector<int> >  */
	/*     simPtsGivenRays(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns); */

	/* void setDeterministicSim(const bool choice); */

	std::vector<TriangleModelSim> m_object_mesh_sims;
	std::vector<std::vector<double> > m_object_centroids;
	std::vector<TriangleModelSim> m_triangle_model_sims;
	std::vector<std::vector<double> > m_imu_posn_nodes;
	std::vector<std::vector<int> > m_block_node_ids_ground;
	LaserCalibParams m_laser_calib_params;

    private:
	double m_max_dist_to_node_for_membership;
	bool m_deterministic_sim;
	double m_object_nn_radius;
    };
}
