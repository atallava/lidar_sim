#pragma once
#include <string>

#include <lidar_sim/SectionLoader.h>

namespace lidar_sim {
    class StateEstPacketAggregator {
    public:
	StateEstPacketAggregator(std::string rel_path_section);
	void aggregatePacketsIntoScans();
	void saveScans(std::string rel_path_data);

    private:
	std::string m_rel_path_section;
	int m_n_packets_per_scan;
	int m_n_packets_skip_between_scans;
	int m_n_scans;

	SectionLoader m_section;
    };
}
