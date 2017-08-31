#pragma once

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/PtsError.h>

namespace lidar_sim {
    class OptimAssistant {
    public:
	OptimAssistant();
	double calcObj(std::vector<double> x);
	void init();
	void buildModelsNonGroundBlock(const int block_id, const std::vector<double> x);
	void sliceSim();
	void blocksSim();
	double calcSimError();

	// todo: so many string helpers! cleanup?
	std::string genRelPathEllipsoids(const int section_id, const int block_id, const int obj_calc_count);
	std::string genRelPathTriangles(int section_id, int block_id);

	// for slice sim
	std::string genRelPathSliceRealPts(const int section_id, const int obj_calc_count);
	std::string genRelPathSliceSimPts(const int section_id, const int obj_calc_count);
	std::string genRelPathSliceSimDetail(const int section_id, const int obj_calc_count);

	// for blocks sim
	std::string genRelPathBlocksRealPts(const int section_id, const int obj_calc_count);
	std::string genRelPathBlocksSimPts(const int section_id, const int obj_calc_count);
	std::string genRelPathBlocksSimDetail(const int section_id, const int obj_calc_count);

	bool m_verbose;
	bool m_initialized;
	int m_section_id_for_model;
	std::string m_rel_path_section_for_model;
	SectionLoader m_section_for_model;
	std::vector<int> m_non_ground_block_ids;
	std::vector<int> m_ground_block_ids;
	int m_section_id_for_sim;
	std::string m_rel_path_section_for_sim;
	SectionLoader m_section_for_sim;

	// for slice sim
	int m_section_packet_start;
	int m_section_packet_end;
	int m_section_packet_step;

	// for blocks sim
	std::vector<int> m_section_pt_ids_for_blocks_sim;
	int m_num_nbrs_for_blocks_sim;
	int m_max_pts_for_blocks_sim;

	std::string m_rel_path_poses_log;
	PoseServer m_imu_pose_server;
	int m_num_nbrs_for_hit_prob;
	PtsError m_error_metric;

    private:
	int m_obj_calc_count;
    };
}
