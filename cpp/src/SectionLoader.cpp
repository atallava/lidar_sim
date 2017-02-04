#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/SectionLoader.h>

using namespace lidar_sim;

SectionLoader::SectionLoader(std::string rel_path_section) :
    m_num_logs(0)
{
    m_loadSection(rel_path_section);
}

void SectionLoader::m_loadSection(std::string rel_path_section)
{
    m_rel_path_section = rel_path_section;
    std::ifstream section_file(rel_path_section);
    if (!section_file)
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "failed to open file " << rel_path_section;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    std::string current_line;
    int prev_packet_id = -1;

    while(std::getline(section_file, current_line))
    {
	std::istringstream iss(current_line);
	int this_packet_id;
	double packet_timestamp_sec;
	double packet_timestamp_nanosec;
	double packet_timestamp;
	
	// packet id and timestamp
	iss >> this_packet_id;
	iss >> packet_timestamp_sec;
	iss >> packet_timestamp_nanosec;
	packet_timestamp = packet_timestamp_sec + packet_timestamp_nanosec*1e-9;

	if (this_packet_id != prev_packet_id)
	{
	    m_packet_ids.push_back(this_packet_id);
	    m_packet_timestamps.push_back(packet_timestamp);
	}

	prev_packet_id = this_packet_id;

	// pt
	std::vector<double> pt(3,0);
	iss >> pt[0];
	iss >> pt[1];
	iss >> pt[2];
	m_pts.push_back(pt);

	m_pt_packet_id.push_back(this_packet_id);
	m_pt_timestamps.push_back(packet_timestamp);
    }
    section_file.close();
    std::cout << "Loaded section: " << rel_path_section << std::endl;
}

