#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdexcept>

#include <boost/algorithm/string.hpp>

#include <lidar_sim/SimDetail.h>
#include <lidar_sim/DataProcessingUtils.h>

using namespace lidar_sim;

SimDetail::SimDetail()
{
}

SimDetail::SimDetail(const std::string rel_path_file)
{
    load(rel_path_file);
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

    std::cout << "Reading sim detail from: " << rel_path_file << std::endl;

    std::string line;
    int count = 0;
    while(std::getline(file, line))
    {
	boost::trim_right(line);
	switch (count % 4)
	{
	case 0:
	{
	    m_ray_origins.push_back(
		getVecFromLine(line));
	    break;
	}
	case 1:
	{
	    m_real_pts.push_back(
		getPtsFromLine(line));
	    break;
	}
	case 2:
	{
	    m_sim_pts.push_back(
		getPtsFromLine(line));
	    break;
	}
	case 3:
	{
	    m_hit_flags.push_back(
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
    std::cout << "Writing sim detail to: " << rel_path_sim_detail << std::endl;
    
    size_t n_poses = m_ray_origins.size();
    for (size_t i = 0; i < n_poses; ++i)
    {
	file << getStrFromVec(m_ray_origins[i]);
	file << getStrFromVec(
	    convertArrayToVec(m_real_pts[i]));
	file << getStrFromVec(
	    convertArrayToVec(m_sim_pts[i]));
	file << getStrFromVec(m_hit_flags[i]);
    }

    file.close();
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

// std::string SimDetail::getStrFromVec(

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

