#pragma once

namespace lidar_sim {
    // ideally state est should be its own package
    namespace algo_state_est {
	std::string genRelPathSectionUnsubsampled(int section_id);
	std::string genRelPathProcessInfo(int section_id, std::string scans_version);
	std::string genRelPathPacketsDir(int section_id, std::string scans_version,
					 std::string data_source, std::string source_version = "");
	bool mkdirsForPacketsToProcess(int section_id, std::string scans_version, 
				       std::string data_source, std::string source_version = "");
	std::string genRelPathPacketsToProcess(int section_id, std::string scans_version,
					       std::string data_source, std::string source_version = "");
	void writeProcessInfo(int section_id, std::string scans_version, 
			      size_t n_scans, size_t n_packets_per_scan, 
			      size_t n_skip_within_scan, size_t n_skip_between_scans);

	bool mkdirsForScans(int section_id, std::string scans_version, 
			    std::string data_source, std::string source_version = "");
	std::string genRelPathScansLaserFrameDir(int section_id, std::string scans_version, 
						 std::string data_source, std::string source_version = "");
	std::string genRelPathScansWorldFrameDir(int section_id, std::string scans_version, 
						 std::string data_source, std::string source_version = "");
	std::string genRelPathScanPoses(int section_id, std::string scans_version, 
					std::string data_source, std::string source_version = "");

	// for the sims
	std::string genRelPathRealPtsRef(int section_id, std::string scans_version, 
					 std::string sim_type, std::string sim_version);
	std::string genRelPathSimPts(int section_id, std::string scans_version, 
				     std::string sim_type, std::string sim_version);
	std::string genRelPathSimDetail(int section_id, std::string scans_version, 
					std::string sim_type, std::string sim_version);
    }
}
