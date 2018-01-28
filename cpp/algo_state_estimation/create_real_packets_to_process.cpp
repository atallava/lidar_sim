// takes an unsubsampled section in world frame, and subsamples it for aggregation into scans
// writes out a file that looks exactly like a section, so that simulating those packets is convenient
// an info file has the parameters of the subsampling scheme

#include <iostream>
#include <ctime>
#include <cmath>
#include <exception>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/AlgoStateEstUtils.h>

using namespace lidar_sim;

std::string genRelPathPacketsToProcess(int section_id, std::string version)
{
    std::ostringstream ss;
    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/version_" << version 
       << "/real/packets_to_process.txt";

    return ss.str();
}

std::string genRelPathProcessInfo(int section_id, std::string version)
{
    std::ostringstream ss;
    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/version_" << version 
       << "/process_info.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // takes about 2 min to read in section_04_world_frame
    // load section
    int section_id = 4;
    std::string rel_path_section = algo_state_est::genRelPathSectionUnsubsampled(section_id);
    SectionLoader section(rel_path_section); 

    size_t n_step_per_scan = 200;
    size_t n_skip_within_scan = 1;
    bool condn = ( std::fmod((double)n_step_per_scan/n_skip_within_scan,10)
		   != 0);
    if (condn)
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "n packets per scan must be an integer.";
	throw std::invalid_argument(ss_err_msg.str().c_str());
    }
    size_t n_packets_per_scan = n_step_per_scan/n_skip_within_scan;
    size_t n_skip_between_scans = 6000;

    // open output file
    std::string datestr_format = "%d%m%y";
    std::string scans_version = getDateString(datestr_format);
    algo_state_est::mkdirsForPacketsToProcess(section_id, scans_version, "real");
    std::string rel_path_packets_to_process = genRelPathPacketsToProcess(section_id, scans_version);
    std::ofstream file_packets_to_process(rel_path_packets_to_process);

    size_t scan_start_idx = 0;
    size_t n_packets = section.m_packet_ids.size();
    int n_scans = 0;
    // loop over scans
    while ( scan_start_idx < (n_packets-1) )
    {
    	size_t scan_end_idx = scan_start_idx + (n_step_per_scan-1);
    	if ( scan_end_idx > (n_packets-1) )
    	    break;

	// loop over packets in scan
    	for (size_t i = scan_start_idx; i <= scan_end_idx; i += n_skip_within_scan)
    	{
	    int packet_id = section.m_packet_ids[i];
	    double t = section.m_packet_timestamps[i];
	    std::vector<std::vector<double> > pts = section.getPtsAtTime(t);
	
	    double intpart, fractpart;
	    fractpart = modf(t, &intpart);
	    int t_sec = (int)intpart;
	    int t_nanosec = (int)(fractpart*1e9);
	    // loop over pts in packet
	    for (size_t j = 0; j < pts.size(); ++j)
	    {
		std::vector<double> pt = pts[j];

		std::ostringstream ss;
		ss << packet_id << " " << t_sec << " " << t_nanosec << " " 
		   << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
		file_packets_to_process << ss.str();
	    }
    	}

	n_scans++;
    	scan_start_idx = n_skip_between_scans + (scan_end_idx + 1);
    }

    std::cout << "Written packets to process to: " << rel_path_packets_to_process << std::endl;
    file_packets_to_process.close();

    // write process info
    std::string rel_path_process_info = genRelPathProcessInfo(section_id, scans_version);
    std::ofstream file_process_info(rel_path_process_info);

    std::ostringstream ss;
    ss << rel_path_section << std::endl;
    file_process_info << ss.str();

    ss.str(""); ss.clear();
    ss << n_scans << " " << std::endl;
    file_process_info << ss.str();

    ss.str(""); ss.clear();
    ss << n_packets_per_scan << " " << std::endl;
    file_process_info << ss.str();

    ss.str(""); ss.clear();
    ss << n_skip_within_scan << " " << std::endl;
    file_process_info << ss.str();

    ss.str(""); ss.clear();
    ss << n_skip_between_scans << " " << std::endl;
    file_process_info << ss.str();

    std::cout << "Written process info to: " << rel_path_process_info << std::endl;
    file_process_info.close();

    double elapsed_time = (clock() - start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s. " << std::endl;

    return(1);
}
