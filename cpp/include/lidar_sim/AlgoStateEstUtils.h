#pragma once

namespace lidar_sim {
    // ideally state est should be its own package
    namespace algo_state_est {
	std::string genRelPathSectionUnsubsampled(int section_id);
	std::string genRelPathPacketsDir(int section_id, std::string scans_version,
					 std::string data_source, std::string source_version = "");
	bool mkdirsForPacketsToProcess(int section_id, std::string scans_version, 
				       std::string data_source, std::string source_version = "");
    }
}
