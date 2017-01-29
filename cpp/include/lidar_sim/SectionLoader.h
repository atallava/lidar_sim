#pragma once
#include <vector>
#include <string>
#include <map>

#include <Eigen/Dense>

namespace lidar_sim {
    class SectionLoader {
    public:
	SectionLoader(std::string rel_path_section);
	std::vector<std::vector<double>> m_pts;

    private:
	void m_loadSection(std::string rel_path_section);
	
	std::vector<int> m_pt_packet_id;
	std::vector<double> m_pt_timestamps;
	std::vector<double> m_packet_timestamps;
	std::vector<int> m_packet_ids;
	std::string m_rel_path_section;
	int m_num_logs;
    };
}

