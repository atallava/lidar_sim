#include <iostream>
#include <iomanip>
#include <sstream>

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
    }
}
