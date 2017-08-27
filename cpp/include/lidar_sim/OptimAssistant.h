#pragma once

namespace lidar_sim {
    class OptimAssistant {
    public:
	OptimAssistant();
	double calcObj(std::vector<double> x);

	bool m_verbose;
	int m_section_id_models;
	std::vector<int> m_non_ground_block_ids;
	std::vector<int> m_ground_block_ids;
	int m_section_id_sim;
	int m_section_packet_start;
	int m_section_packet_end;
	int m_section_packet_skip;

    private:
    };
}
