#include <lidar_sim/StateEstPacketAggregator.h>

using namespace lidar_sim;


StateEstPacketAggregator::StateEstPacketAggregator(std::string rel_path_section) : 
    m_n_packets_per_scan(200),
    m_n_packets_skip_between_scans(10),
    m_section(rel_path_section)
{
    // SectionLoader section(rel_path_section);
    // m_section = section;
}

void StateEstPacketAggregator::aggregatePacketsIntoScans()
{
    m_n_scans = 0;
    
    size_t n_packets = m_section.m_packet_ids.size();

    size_t scan_start_idx = 0;
    while (scan_start_idx < (n_packets-1)) 
    {
	size_t scan_end_idx += scan_start_idx + n_packets_per_scan;
	if (scan_end_idx > (n_packets-1))
	    break;
	
	for (size_t i = scan_start_idx; i <= scan_end_idx; ++i)
	{
	    double t = m_section.m_packet_timestamps[i];
	    std::vector<std::vector<double> > packet_pts = m_section.getPtsAtTime(t);

	    // transform into world frame
	    // aggreate into scan pts
	}
    }
}

void StateEstPacketAggregator::saveScans(std::string rel_path_data)
{
}
