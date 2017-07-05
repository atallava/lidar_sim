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
	pitch_vec[i] *= 180/M_PI;

    m_pattern_pitch_vec = pitch_vec;

    m_n_pattern_yaw_vec = 12;
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
    std::vector<double> pattern_yaw_vec = linspace(pattern_yaw_start, pattern_yaw_end, m_n_pattern_yaw_vec);

    // todo: delete me
    std::cout << "pattern yaw start, end: " << pattern_yaw_start << " " << pattern_yaw_end << std::endl;
    std::cout << "pattern yaw vec: " << std::endl;
    dispVec(pattern_yaw_vec);
    
    return pattern_yaw_vec;
}

