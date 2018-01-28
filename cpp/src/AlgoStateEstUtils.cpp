#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

// hacking boost filesystem bug
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <lidar_sim/AlgoStateEstUtils.h>

namespace lidar_sim {
    namespace algo_state_est {

	std::string genRelPathSectionUnsubsampled(int section_id)
	{
	    std::ostringstream ss;
	    ss << "../data/taylorJune2014/sections/world_frame/section_" << std::setw(2) << std::setfill('0') << section_id 
	       << "_world_frame.xyz";

	    return ss.str();
	}

	std::string genRelPathProcessInfo(int section_id, std::string scans_version)
	{
	    std::ostringstream ss;
	    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
	       << "/version_" << scans_version;
	    std::string rel_path_dir = ss.str();
	    std::string rel_path_info = rel_path_dir + "/process_info.txt";

	    return rel_path_info;
	}

	std::string genRelPathPacketsDir(int section_id, std::string scans_version,
					 std::string data_source, std::string source_version)
	{
	    std::ostringstream ss;
	    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
	       << "/version_" << scans_version
	       << "/" << data_source;
	    if (source_version != "")
		ss << "/version_" << source_version;
	    std::string rel_path_dir = ss.str();

	    return rel_path_dir;
	}


	bool mkdirsForPacketsToProcess(int section_id, std::string scans_version, 
				       std::string data_source, std::string source_version)
	{
	    std::string rel_path_dir = genRelPathPacketsDir(section_id, scans_version, data_source, source_version);
	    return boost::filesystem::create_directories(rel_path_dir);
	}

	std::string genRelPathPacketsToProcess(int section_id, std::string scans_version,
					       std::string data_source, std::string source_version)
	{
	    std::string rel_path_dir = genRelPathPacketsDir(section_id, scans_version, data_source, source_version);
	    std::string rel_path_packets = rel_path_dir + "/packets_to_process.xyz";

	    return rel_path_packets;
	}

	void writeProcessInfo(int section_id, std::string scans_version, 
			      size_t n_scans, size_t n_packets_per_scan, 
			      size_t n_skip_within_scan, size_t n_skip_between_scans)
	{
	    std::string rel_path_process_info = 
		genRelPathProcessInfo(section_id, scans_version);
	    std::ofstream file_process_info(rel_path_process_info);

	    std::ostringstream ss;
	    ss << n_scans << std::endl;
	    file_process_info << ss.str();

	    ss.str(""); ss.clear();
	    ss << n_packets_per_scan << std::endl;
	    file_process_info << ss.str();

	    ss.str(""); ss.clear();
	    ss << n_skip_within_scan << std::endl;
	    file_process_info << ss.str();

	    ss.str(""); ss.clear();
	    ss << n_skip_between_scans << std::endl;
	    file_process_info << ss.str();

	    std::cout << "Written process info to: " << rel_path_process_info << std::endl;
	    file_process_info.close();
	}

	std::string genRelPathRealPtsRef(int section_id, std::string scans_version, 
					 std::string sim_type, std::string sim_version)
	{
	    std::ostringstream ss;
	    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
	       << "/version_" << scans_version 
	       << "/" << sim_type << "/version_" << sim_version
	       << "real_pts.xyz";
	    std::string rel_path_dir = ss.str();
	    return ss.str();
	}

	std::string genRelPathSimPts(int section_id, std::string scans_version, 
				     std::string sim_type, std::string sim_version)
	{
	    std::ostringstream ss;
	    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
	       << "/version_" << scans_version 
	       << "/" << sim_type << "/version_" << sim_version
	       << "sim_pts.xyz";
	    std::string rel_path_dir = ss.str();
	    return ss.str();
	}

	std::string genRelPathSimDetail(int section_id, std::string scans_version, 
					std::string sim_type, std::string sim_version)
	{
	    std::ostringstream ss;
	    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
	       << "/version_" << scans_version 
	       << "/" << sim_type << "/version_" << sim_version
	       << "sim_detail.txt";
	    std::string rel_path_dir = ss.str();
	    return ss.str();
	}
    }
}
