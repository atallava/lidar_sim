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

using namespace lidar_sim;

RayDirnServer::RayDirnServer()
{
    std::vector<double> pitch_vec{-30.67,-9.33,-29.33,-8,-28,-6.66,-26.66,-5.33,-25.33,-4,-24,-2.67,-
	    22.67,-1.33,-21.33,0,-20,1.33,-18.67,2.67,-17.33,4,-16,5.33,-14.67,6.67,-13.33,8,-12,9.33,-10.67,10.67};
    std::sort(pitch_vec.begin(), pitch_vec.end());
    // convert to rad
    for (size_t i = 0; i < pitch_vec.size(); ++i)
	pitch_vec[i] *= 180/M_PI;

    m_pitch_vec = pitch_vec;
}

std::vector<double> RayDirnServer::getRayYawPitchVec(const std::vector<double> &origin, 
						     const std::vector<std::vector<double> > &pts)
{
    size_t n_pts = pts.size();
    std::vector<std::vector<double> > ray_dirns  = calcRayDirns(origin, pts);
    std::vector<double> ray_yaw_vec(n_pts, 0);
    std::vector<double> ray_pitch_vec(n_pts, 0);
    for (size_t i = 0; i < n_pts; ++i)
	std::tie(ray_yaw_vec[i], ray_pitch_vec[i], std::ignore) = 
	    cart2sph(ray_dirns[i]);

    return ray_yaw_vec;
}


