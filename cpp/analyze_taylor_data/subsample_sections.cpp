#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <cstring>
#include <math.h>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/SectionLoader.h>

using namespace lidar_sim;

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    ss << "../data/taylorJune2014/sections/world_frame/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame.xyz";

    return ss.str();
}

std::string genRelPathSectionSubsampled(int section_id)
{
    std::ostringstream ss;
    ss << "../data/taylorJune2014/sections/world_frame/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_subsampled.xyz";

    return ss.str();
}

int main() {
    std::vector<int> section_ids;
    for (size_t i = 1; i <= 14; ++i) 
	section_ids.push_back(i);
    
    clock_t start_time = clock();

    for (size_t i = 0; i < section_ids.size(); ++i)
    {
	int section_id = section_ids[i];
	std::string rel_path_section = genRelPathSection(section_id);
	SectionLoader section(rel_path_section);

	std::string rel_path_section_subsampled = genRelPathSectionSubsampled(section_id);
	std::ofstream file_section_subsampled(rel_path_section_subsampled);

	int step = 30;
	size_t count = 0;
	size_t n_packets = section.m_packet_timestamps.size();
	while (count < n_packets)
	{
	    int packet_id = section.m_packet_ids[count];
	    double t = section.m_packet_timestamps[count];
	    std::vector<std::vector<double> > pts = section.getPtsAtTime(t);
	
	    double intpart, fractpart;
	    fractpart = modf(t, &intpart);
	    int t_sec = (int)intpart;
	    int t_nanosec = (int)(fractpart*1e9);
	    for (size_t j = 0; j < pts.size(); ++j)
	    {
		std::vector<double> pt = pts[j];

		std::ostringstream ss;
		ss << packet_id << " " << t_sec << " " << t_nanosec << " " 
		   << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
		file_section_subsampled << ss.str();
	    }

	    count += step;
	}
	std::cout << "Written subsampled section to: " << rel_path_section_subsampled << std::endl;
	file_section_subsampled.close();
    } 

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
