#pragma once
#include <vector>
#include <string>
#include <map>

#include <Eigen/Dense>

#include <lidar_sim/PoseServer.h>

namespace lidar_sim {
    class SectionLoader {
    public:
	SectionLoader(std::string rel_path_section);
	std::vector<std::vector<double> > getPtsAtTime(double t);
	std::vector<int> getPtIdsAtTime(double t);
	std::vector<std::vector<double> > getSectionImuPoses(const PoseServer &imu_pose_server);
	std::vector<std::vector<double> > getSectionImuPosns(const PoseServer &imu_pose_server);
	// 'log' corresponds to the packet timestamps vec
	std::tuple<int, int> getLogIdsBracketingImuPosns(const std::vector<std::vector<double> > &posns, const PoseServer &imu_pose_server);

	std::vector<std::vector<double>> m_pts;
	std::vector<double> m_pt_timestamps;
	std::vector<double> m_packet_timestamps;
	std::vector<int> m_packet_ids;

    private:
	void loadSection(std::string rel_path_section);
	
	std::vector<int> m_pt_packet_id;
	std::string m_rel_path_section;
	int m_num_logs;
	int m_bracketing_padding;
    };
}
