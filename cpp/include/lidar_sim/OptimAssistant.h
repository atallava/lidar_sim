#pragma once

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/PtsError.h>
#include <lidar_sim/LaserCalibParams.h>

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
	void fillSectionPtIdsForBlocksSim();
	void createSimDetailTemplate();

	// relpath helpers
	std::string genRelPathEllipsoids(const int section_id, const int block_id, const int obj_calc_count);
	std::string genRelPathTriangles(int section_id, int block_id);
	std::string genRelPathRealPts(const int section_id, const int obj_calc_count);
	std::string genRelPathSimPts(const int section_id, const int obj_calc_count);
	std::string genRelPathSimDetail(const int section_id, const int obj_calc_count);

	bool m_verbose;
	bool m_initialized;
	std::string m_instance_idx;

	int m_section_id_for_model;
	std::string m_rel_path_section_for_model;
	SectionLoader m_section_for_model;
	std::vector<int> m_non_ground_block_ids;
	std::vector<int> m_ground_block_ids;

	int m_sim_type; // 0 for blocks, 1 for slice
	int m_section_id_for_sim;
	std::string m_rel_path_section_for_sim;
	SectionLoader m_section_for_sim;
	double m_slice_hit_to_blocks_threshold; // todo: this really needs a better name
	double m_slice_resn_along_ray;
	double m_slice_miss_to_blocks_threshold;

	// for blocks sim
	std::vector<int> m_section_pt_ids_for_blocks_sim;
	int m_num_nbrs_for_blocks_sim;
	int m_max_pts_for_blocks_sim;

	// for slice sim
	SimDetail m_sim_detail_template;
	int m_section_packet_start;
	int m_section_packet_end;
	int m_section_packet_step;

	std::string m_rel_path_poses_log;
	PoseServer m_imu_pose_server;
	LaserCalibParams m_laser_calib_params;
	int m_num_nbrs_for_hit_prob;
	PtsError m_error_metric;
	
    private:
	int m_obj_calc_count;
    };
}
