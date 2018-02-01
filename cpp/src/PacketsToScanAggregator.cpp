#include <iostream>
#include <fstream>
#include <stdexcept>
#include <iomanip>

#include <lidar_sim/PacketsToScanAggregator.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

PacketsToScanAggregator::PacketsToScanAggregator() :
    m_verbose(0)
{
    std::string path_poses_log = genPathPosesLog();
    m_imu_pose_server = PoseServer (path_poses_log);
}

void PacketsToScanAggregator::loadPackets(std::string rel_path_packets)
{
    if (m_verbose)
	std::cout << "Reading packets from " << rel_path_packets << std::endl;
    m_packets = SectionLoader(rel_path_packets);
}

void PacketsToScanAggregator::loadProcessInfo(std::string rel_path_process_info)
{
    if (m_verbose)
	std::cout << "Reading process info from " << rel_path_process_info << std::endl;

    std::ifstream file(rel_path_process_info);
    if (!file)
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "failed to open file " << rel_path_process_info;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    std::string current_line;
    std::getline(file, current_line); // number of scans
    std::istringstream iss_1(current_line);
    iss_1 >> m_n_scans_expected;
    std::getline(file, current_line); // n packets per scan
    std::istringstream iss_2(current_line);
    iss_2 >> m_n_packets_per_scan;
    // ignore other data 
    file.close();
}

void PacketsToScanAggregator::aggregate()
{
    size_t n_packets = m_packets.m_packet_ids.size();

    m_n_scans = 0;
    size_t scan_start_idx = 0;
    while (scan_start_idx < n_packets) 
    {
	size_t scan_end_idx = scan_start_idx + (m_n_packets_per_scan - 1);
	if (scan_end_idx >= n_packets)
	    break;

	Pts scan; // by default in world frame
	std::vector<double> scan_packet_timestamps;
	for (size_t i = scan_start_idx; i <= scan_end_idx; ++i)
	{
	    double t = m_packets.m_packet_timestamps[i];
	    Pts packet_pts = m_packets.getPtsAtTime(t);

	    scan.insert(scan.end(), 
			packet_pts.begin(), packet_pts.end());
	    scan_packet_timestamps.push_back(t);
	}
	m_scans_world_frame.push_back(scan);

	// timestamp for scan
	double scan_timestamp;
	std::tie(scan_timestamp, std::ignore) = calcVecMeanVar(scan_packet_timestamps);
	m_scan_timestamps.push_back(scan_timestamp);

	// laser pose
	Eigen::Matrix4d T_imu = m_imu_pose_server.getTransfAtTime(scan_timestamp);
	Eigen::Matrix4d T_laser = T_imu*
	    m_laser_calib_params.extrinsics.T_laser_imu;
	m_laser_poses.push_back(T_laser);

	// scan in laser frame
	Eigen::Matrix4d T_world_laser = T_imu.inverse();
	Pts scan_laser_frame = applyTransfToPts(scan, T_world_laser);
	m_scans_laser_frame.push_back(scan_laser_frame);

	m_n_scans++;
	scan_start_idx = scan_end_idx + 1;
    }

    bool condn = (m_n_scans == m_n_scans_expected);
    if (!condn)
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "n scans expected: " << m_n_scans_expected 
		   << " doesn't equal n scans aggregated: " << m_n_scans << std::endl;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }
}

void PacketsToScanAggregator::saveScansWorldFrame(std::string rel_path_scans_dir)
{
    for (size_t i = 0; i < (size_t)m_n_scans; ++i)
    {
	std::stringstream ss;
	int width_arg = getNumDigits(m_n_scans);
	ss << rel_path_scans_dir << "/scan_" << std::setw(width_arg) << std::setfill('0') << i 
	   << ".xyz";
	std::string rel_path_scan = ss.str();

	std::ofstream file_scan(rel_path_scan);
	Pts pts = m_scans_world_frame[i];

	for (size_t j = 0; j < pts.size(); ++j)
	{
	    std::vector<double> pt = pts[j];
	    std::stringstream oss;
	    oss << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
	    file_scan << oss.str();
	}
	file_scan.close();
    }
}

void PacketsToScanAggregator::saveScansLaserFrame(std::string rel_path_scans_dir)
{
    for (size_t i = 0; i < (size_t)m_n_scans; ++i)
    {
	std::stringstream ss;
	int width_arg = getNumDigits(m_n_scans);
	ss << rel_path_scans_dir << "/scan_" << std::setw(width_arg) << std::setfill('0') << i 
	   << ".xyz";
	std::string rel_path_scan = ss.str();

	std::ofstream file_scan(rel_path_scan);
	Pts pts = m_scans_laser_frame[i];
	for (size_t j = 0; j < pts.size(); ++j)
	{
	    std::vector<double> pt = pts[j];
	    std::stringstream oss;
	    oss << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
	    file_scan << oss.str();
	}
	file_scan.close();
    }
}

void PacketsToScanAggregator::saveScanPoses(std::string rel_path_scan_poses)
{
    std::ofstream file_scan_poses(rel_path_scan_poses);
    
    for (size_t i = 0; i < (size_t)m_n_scans; ++i)
    {
	double t = m_scan_timestamps[i];
	double intpart, fractpart;
	fractpart = modf(t, &intpart);
	int t_sec = (int)intpart;
	int t_nanosec = (int)(fractpart*1e9);

	Eigen::Matrix4d T = m_laser_poses[i];

	std::stringstream oss;
	oss << t_sec << " " << t_nanosec << " ";
	// transform written in column-major order
	for (size_t col = 0; col < 4; ++col)
	    for (size_t row = 0; row < 4; ++row)
		oss << T(row,col) << " ";

	oss << std::endl;
	file_scan_poses << oss.str();
    }
}

void PacketsToScanAggregator::setVerbosity(int verbose)
{
    m_verbose = verbose;
}
