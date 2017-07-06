#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdexcept>
#include <math.h>

#include <boost/algorithm/string.hpp>

#include <lidar_sim/RayDirnServer.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

RayDirnServer::RayDirnServer()
{
    std::vector<double> pitch_vec{-30.67,-9.33,-29.33,-8,-28,-6.66,-26.66,-5.33,-25.33,-4,-24,-2.67,-
	    22.67,-1.33,-21.33,0,-20,1.33,-18.67,2.67,-17.33,4,-16,5.33,-14.67,6.67,-13.33,8,-12,9.33,-10.67,10.67};
    std::sort(pitch_vec.begin(), pitch_vec.end());
    // convert to rad
    for (size_t i = 0; i < pitch_vec.size(); ++i)
	pitch_vec[i] *= M_PI/180;

    m_pattern_pitch_vec = pitch_vec;
    m_n_pattern_pitches = m_pattern_pitch_vec.size();

    m_n_pattern_yaws = 12;

    m_min_pts_to_fit_detail = 75;
}

std::tuple<std::vector<double>, std::vector<double> >
	   RayDirnServer::getRayYawPitchVec(const std::vector<double> &origin, 
					    const std::vector<std::vector<double> > &pts)
{
    size_t n_pts = pts.size();
    std::vector<std::vector<double> > ray_dirns  = calcRayDirns(origin, pts);
    std::vector<double> ray_yaw_vec(n_pts, 0);
    std::vector<double> ray_pitch_vec(n_pts, 0);
    for (size_t i = 0; i < n_pts; ++i)
	std::tie(ray_yaw_vec[i], ray_pitch_vec[i], std::ignore) = 
	    cart2sph(ray_dirns[i]);

    return std::tie(ray_yaw_vec, ray_pitch_vec);
}

std::vector<double> RayDirnServer::getPatternYawVec(const std::vector<double> &ray_yaw_vec)
{
    auto min_it = std::min_element(ray_yaw_vec.begin(), ray_yaw_vec.end());
    size_t min_posn = std::distance(ray_yaw_vec.begin(), min_it);
    double ray_yaw_min = ray_yaw_vec[min_posn];

    auto max_it = std::max_element(ray_yaw_vec.begin(), ray_yaw_vec.end());
    size_t max_posn = std::distance(ray_yaw_vec.begin(), max_it);
    double ray_yaw_max = ray_yaw_vec[max_posn];

    size_t n_rays = ray_yaw_vec.size();
    // yaw start and end
    std::vector<int> flag_min(n_rays, 0);
    std::vector<int> flag_max(n_rays, 0);
    for (size_t i = 0; i < n_rays; ++i)
    {
	bool condn1 = (ray_yaw_vec[i] >= ray_yaw_min);
	bool condn2 = (ray_yaw_vec[i] <= (ray_yaw_min + 1e-3));
	if (condn1 && condn2)
	    flag_min[i] = 1;

	condn1 = (ray_yaw_vec[i] >= (ray_yaw_max - 1e-3));
	condn2 = (ray_yaw_vec[i] <= ray_yaw_max);
	if (condn1 && condn2)
	    flag_max[i] = 1;
    }
    double pattern_yaw_start;
    std::tie(pattern_yaw_start, std::ignore) = calcVecMeanVar(
	logicalSubsetArray(ray_yaw_vec, flag_min));
    double pattern_yaw_end;
    std::tie(pattern_yaw_end, std::ignore) = calcVecMeanVar(
	logicalSubsetArray(ray_yaw_vec, flag_max));

    // initial estimate
    std::vector<double> pattern_yaw_vec = linspace(pattern_yaw_start, pattern_yaw_end, m_n_pattern_yaws);

    // map each point to closest yaw
    std::vector<double> yaw_membership_ids(n_rays, 0);
    for (size_t i = 0; i < n_rays; ++i)
    {
	std::vector<double> dists(m_n_pattern_yaws, 0);
	for (size_t j = 0; j < (size_t)m_n_pattern_yaws; ++j)
	    dists[j] = std::abs(ray_yaw_vec[i] - pattern_yaw_vec[j]);
	
	auto min_it = std::min_element(dists.begin(), dists.end());
	size_t min_posn = std::distance(dists.begin(), min_it);
	
	yaw_membership_ids[i] = min_posn;
    }
   
    // update pattern yaws
    for (size_t i = 0; i < (size_t)m_n_pattern_yaws; ++i)
    {
	std::vector<double> this_bucket_yaws;
	for (size_t j = 0; j < n_rays; ++j)
	    if (yaw_membership_ids[j] == i)
		this_bucket_yaws.push_back(ray_yaw_vec[j]);
	
	if (this_bucket_yaws.size() > 0)
	    std::tie(pattern_yaw_vec[i], std::ignore) = 
		calcVecMeanVar(this_bucket_yaws);
    }

    // debug
    // std::cout << "pattern yaw start, end: " << pattern_yaw_start << " " << pattern_yaw_end << std::endl;
    // std::cout << "init pattern yaw vec: " << std::endl;
    // dispVec(linspace(pattern_yaw_start, pattern_yaw_end, m_n_pattern_vec));
    // std::cout << "yaw membership ids: " << std::endl;
    // dispVec(yaw_membership_ids);

    return pattern_yaw_vec;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<std::vector<double> >, std::vector<int> >
RayDirnServer::fitDetailToPts(const std::vector<double> &origin, 
			      const std::vector<std::vector<double> > &pts)
{
    if (pts.size() < (size_t)m_min_pts_to_fit_detail)
	return defaultDetail(origin, pts);

    std::vector<double> ray_yaw_vec, ray_pitch_vec;
    std::tie(ray_yaw_vec, ray_pitch_vec) = getRayYawPitchVec(origin, pts);
    size_t n_rays = pts.size();

    std::vector<double> pattern_yaw_vec = getPatternYawVec(ray_yaw_vec);

    // yaw membership ids
    std::vector<double> yaw_membership_ids(n_rays, 0);
    for (size_t i = 0; i < n_rays; ++i)
    {
	std::vector<double> dists(m_n_pattern_yaws, 0);
	for (size_t j = 0; j < (size_t)m_n_pattern_yaws; ++j)
	    dists[j] = std::abs(ray_yaw_vec[i] - pattern_yaw_vec[j]);
	
	auto min_it = std::min_element(dists.begin(), dists.end());
	size_t min_posn = std::distance(dists.begin(), min_it);
	
	yaw_membership_ids[i] = min_posn;
    }
   

    // pitch membership ids
    std::vector<double> pitch_membership_ids(n_rays, 0);
    for (size_t i = 0; i < n_rays; ++i)
    {
	std::vector<double> dists(m_n_pattern_pitches, 0);
	for (size_t j = 0; j < (size_t)m_n_pattern_pitches; ++j)
	    dists[j] = std::abs(ray_pitch_vec[i] - m_pattern_pitch_vec[j]);
	
	auto min_it = std::min_element(dists.begin(), dists.end());
	size_t min_posn = std::distance(dists.begin(), min_it);
	
	pitch_membership_ids[i] = min_posn;
    }
   

    size_t n_unrolled_pts = m_n_pattern_pitches*m_n_pattern_yaws;
    std::vector<double> unrolled_yaws(n_unrolled_pts, 0);
    std::vector<double> unrolled_pitches(n_unrolled_pts, 0);
    std::vector<std::vector<double> > unrolled_pts(n_unrolled_pts, 
						   std::vector<double>(3, 0));
    std::vector<int> unrolled_hit_flag(n_unrolled_pts, 0);

    size_t count = 0;
    for (size_t i = 0; i < (size_t)m_n_pattern_yaws; ++i)
	for (size_t j = 0; j < (size_t)m_n_pattern_pitches; ++j)
	{
	    unrolled_yaws[count] = pattern_yaw_vec[i];
	    unrolled_pitches[count] = m_pattern_pitch_vec[j];

	    // does any point belong here
	    std::vector<int> ids;
	    for (size_t k = 0; k < pts.size(); ++k)
	    {
		bool condn1 = (yaw_membership_ids[k] == i);
		bool condn2 = (pitch_membership_ids[k] == j);
		if (condn1 && condn2)
		    ids.push_back(k);
	    }
	    
	    // if no point belongs here, hit flag = 0
	    if (ids.empty())
		unrolled_hit_flag[count] = 0;
	    else
	    {
		size_t id = ids[0];
		unrolled_hit_flag[count] = 1;
		unrolled_pts[count] = pts[id];
	    }
	    count ++;
	}

    return std::make_tuple(unrolled_pitches, unrolled_yaws, unrolled_pts, unrolled_hit_flag);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<std::vector<double> >, std::vector<int> >
RayDirnServer::defaultDetail(const std::vector<double> &origin, 
			     const std::vector<std::vector<double> > &pts)
{
    std::vector<double> ray_yaw_vec, ray_pitch_vec;
    std::tie(ray_yaw_vec, ray_pitch_vec) = getRayYawPitchVec(origin, pts);
    size_t n_rays = pts.size();
    std::vector<int> hit_flag(n_rays, 1); // all are hits
    return std::make_tuple(ray_pitch_vec, ray_yaw_vec, pts, hit_flag);
}
