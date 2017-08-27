#pragma once

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/SectionLoader.h>

namespace lidar_sim {
    class OptimAssistant {
    public:
	OptimAssistant();
	double calcObj(std::vector<double> x);
	void init();
	void buildModelsNonGroundBlock(const int block_id, const std::vector<double> x);
	void simulate();

	// todo: so many string helpers! cleanup?
	std::string genRelPathBlockPts(const int section_id, const int block_id);
	std::string genRelPathSection(const int section_id);
	std::string genRelPathEllipsoids(const int section_id, const int block_id);
	std::string genRelPathTriangles(int section_id, int block_id);
	std::string genRelPathImuPosnNodes(int section_id);
	std::string genRelPathBlockNodeIdsGround(int section_id);
	std::string genRelPathBlockNodeIdsNonGround(int section_id);
	std::string genRelPathSliceRealPts(int section_id);
	std::string genRelPathSimPts(int section_id);
	std::string genRelPathSimDetail(int section_id);

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
	int m_section_packet_start;
	int m_section_packet_end;
	int m_section_packet_step;
	std::string m_rel_path_poses_log;
	PoseServer m_imu_pose_server;
	int m_num_nbrs_for_hit_prob;

    private:
    };
}
