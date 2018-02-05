#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdexcept>

#include <boost/algorithm/string.hpp>

#include <lidar_sim/SimDetail.h>
#include <lidar_sim/DataProcessingUtils.h>

using namespace lidar_sim;

SimDetail::SimDetail() :
    m_verbose(1)
{
}

SimDetail::SimDetail(const std::string rel_path_file) :
    m_verbose(1)
{
    load(rel_path_file);
}

void SimDetail::reserve(const int size)
{
    m_packet_ids.reserve(size);
    m_packet_timestamps.reserve(size);
    m_ray_origins.reserve(size);
    m_ray_pitches.reserve(size);
    m_ray_yaws.reserve(size);
    m_real_pts_all.reserve(size);
    m_real_hit_flags.reserve(size);
    m_sim_pts_all.reserve(size);
    m_sim_hit_flags.reserve(size);
}

void SimDetail::load(const std::string rel_path_file)
{
    std::ifstream file(rel_path_file);
    if (!file)
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "failed to open file " << rel_path_file;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    if (m_verbose)
	std::cout << "Reading sim detail from: " << rel_path_file << std::endl;

    std::string line;
    int count = 0;
    while(std::getline(file, line))
    {
	boost::trim_right(line);
	switch (count % 7)
	{
	case 0:
	{
	    m_ray_origins.push_back(
		getVecFromLine(line));
	    break;
	}
	case 1:
	{
	    m_ray_pitches.push_back(
		getVecFromLine(line));
	    break;
	}
	case 2:
	{
	    m_ray_yaws.push_back(
		getVecFromLine(line));
	    break;
	}
	case 3:
	{
	    m_real_pts_all.push_back(
		getPtsFromLine(line));
	    break;
	}
	case 4:
	{
	    m_real_hit_flags.push_back(
		getHitFlagFromLine(line));
	    break;
	}
	case 5:
	{
	    m_sim_pts_all.push_back(
		getPtsFromLine(line));
	    break;
	}
	case 6:
	{
	    m_sim_hit_flags.push_back(
		getHitFlagFromLine(line));
	    break;
	}
	}

	count++;
    }

}

void SimDetail::save(const std::string rel_path_sim_detail)
{
    std::ofstream file(rel_path_sim_detail);
    if (m_verbose)
	std::cout << "Writing sim detail to: " << rel_path_sim_detail << std::endl;
    
    size_t n_poses = m_ray_origins.size();
    for (size_t i = 0; i < n_poses; ++i)
    {
	// ray origin
	file << getStrFromVec(m_ray_origins[i]) << std::endl;
	// pitches
	file << getStrFromVec(m_ray_pitches[i]) << std::endl;
	// yaws
	file << getStrFromVec(m_ray_yaws[i]) << std::endl;
	// real pts all
	file << getStrFromVec(
	    convertArrayToVec(m_real_pts_all[i])) << std::endl;
	// real hit flag
	file << getStrFromVec(m_real_hit_flags[i]) << std::endl;
	// sim pts all
	file << getStrFromVec(
	    convertArrayToVec(m_sim_pts_all[i])) << std::endl;
	// sim hit flag
	file << getStrFromVec(m_sim_hit_flags[i]) << std::endl;
    }

    file.close();
}

void SimDetail::writeSimPackets(const std::string rel_path_sim_packets)
{
    std::ofstream file_sim_packets(rel_path_sim_packets);

    for (size_t i = 0; i < m_packet_ids.size(); ++i)
    {
	int packet_id = m_packet_ids[i];
	double t = m_packet_timestamps[i];
	double intpart, fractpart;
	fractpart = modf(t, &intpart);
	int t_sec = (int)intpart;
	int t_nanosec = (int)(fractpart*1e9);
	std::vector<std::vector<double> > this_sim_pts_all 
	    = m_sim_pts_all[i];
	std::vector<int> this_sim_hit_flag
	    = m_sim_hit_flags[i];
	for (size_t j = 0; j < this_sim_pts_all.size(); ++j)
	{
	    if (!this_sim_hit_flag[j]) 
		continue;

	    std::vector<double> pt = this_sim_pts_all[j];
	    std::ostringstream ss;
	    ss << packet_id << " " << t_sec << " " << t_nanosec << " " 
	       << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
	    file_sim_packets << ss.str();
	}
    }
    std::cout << "Written sim packets to process to: " << rel_path_sim_packets  << std::endl;
    file_sim_packets.close();
}

void SimDetail::setVerbosity(int verbose)
{
    m_verbose = verbose;
}

std::vector<double> SimDetail::getVecFromLine(const std::string line)
{
    std::istringstream iss(line);
    std::vector<double> vec;
    while (!iss.eof())
    {
	double val;
	iss >> val;
	vec.push_back(val);
    }

    return vec;
}

std::vector<std::vector<double> > SimDetail::getPtsFromLine(const std::string line)
{
    std::vector<double> vec = getVecFromLine(line);
    size_t num_pts = vec.size()/3;

    size_t count = 0;
    std::vector<std::vector<double> > pts;
    for(size_t i = 0; i < num_pts; ++i)
    {
	std::vector<double> pt;
	for(size_t j = 0; j < 3; ++j)
	{
	    pt.push_back(vec[count]);
	    count++;
	}
	pts.push_back(pt);
    }

    return pts;
}

std::vector<int> SimDetail::getHitFlagFromLine(const std::string line)
{
    std::istringstream iss(line);
    std::vector<int> vec;
    while (!iss.eof())
    {
	int val;
	iss >> val;
	vec.push_back(val);
    }

    return vec;
}

