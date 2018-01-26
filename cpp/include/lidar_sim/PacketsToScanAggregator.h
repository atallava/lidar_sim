#pragma once
#include <string>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    class PacketsToScanAggregator {
    public:
	PacketsToScanAggregator();
	void loadPackets(std::string rel_path_packets);
	void loadProcessInfo(std::string rel_path_process_info);
	void aggregate();
	void saveScansWorldFrame(std::string rel_path_scans_dir);
	void saveScansLaserFrame(std::string rel_path_scans_dir);
	void saveScanPoses(std::string rel_path_scan_poses);
	void setVerbosity(int verbose);

	typedef std::vector<std::vector<double> > Pts;

    private:
	int m_verbose;
	SectionLoader m_packets;
	PoseServer m_imu_pose_server;
	LaserCalibParams m_laser_calib_params;
	int m_n_packets_per_scan;
	
	int m_n_scans;
	std::vector<Pts> m_scans_world_frame;
	std::vector<Pts> m_scans_laser_frame;
	std::vector<Eigen::Matrix4d> m_laser_poses;
	std::vector<double> m_scan_timestamps;
    };
}
