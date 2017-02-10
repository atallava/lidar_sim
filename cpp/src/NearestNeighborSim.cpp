#include <algorithm>
#include <random>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/NearestNeighborSim.h>

using namespace lidar_sim;

NearestNeighborSim::NearestNeighborSim() :
    m_debug_flag(0),
    m_max_perp_dist_for_hit(0.1),
    m_range_var(0.2),
    m_max_pts(1e5)
{    
}

// load pts
void NearestNeighborSim::loadPts(const std::string rel_path_pts)
{
    m_pts = loadPtsFromXYZFile(rel_path_pts);

    // todo: subsample or not
    subsamplePts();
}

void NearestNeighborSim::setDebugFlag(const int value)
{
    m_debug_flag = value;
}

void NearestNeighborSim::subsamplePts()
{
    if (m_debug_flag)
	std::cout << "NearestNeighborSim: subsampling pts... " << std::endl;

    if (m_pts.empty())
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "NearestNeighborSim: m_pts is empty!";
	throw std::runtime_error(ss_err_msg.str().c_str());
    }
	
    int skip = (int)(m_pts.size()/m_max_pts);
    std::vector<std::vector<double> > pts_sub;
    for(size_t i = 0; i < m_pts.size(); i += skip)
	pts_sub.push_back(m_pts[i]);

    m_pts = pts_sub;
}

// sim pts given pose
std::tuple<std::vector<std::vector<double> >, std::vector<int> >
NearestNeighborSim::simPtsGivenPose(const std::vector<double> &imu_pose)
{
    // get ray origin and ray dirn
    std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, m_laser_calib_params);
    std::vector<std::vector<double> > ray_dirns = genRayDirnsWorldFrame(imu_pose, m_laser_calib_params);
    
    std::vector<std::vector<double> > sim_pts(ray_dirns.size(), std::vector<double>(3,0));
    std::vector<int> hit_flag(ray_dirns.size(),0);

    for(size_t i = 0; i < ray_dirns.size(); ++i)
    {
	std::vector<double> dist_along_ray(m_pts.size(),0);
	std::vector<double> perp_dist(m_pts.size(),0);
	int closest_id = -1;
	double min_dist_along_ray = m_laser_calib_params.intrinsics.max_range;

	for(size_t j = 0; j < m_pts.size(); ++j)
	{
	    try
	    {
		dist_along_ray[j] = dotProduct(ray_dirns[i], vectorDiff(m_pts[j], ray_origin));
	    }
	    catch (const std::exception& e)
	    {
		std::cout << "error" << std::endl;
		exit(0);
	    }
	    
	    std::vector<double> pt_projn(3,0);
	    for(size_t k = 0; k < 3; ++k)
		pt_projn[k] = ray_origin[k] + dist_along_ray[j]*ray_dirns[i][k];
	    perp_dist[j] = vectorNorm(vectorDiff(pt_projn, m_pts[j]));

	    bool condn1 = (dist_along_ray[j] >= m_laser_calib_params.intrinsics.min_range);
	    bool condn2 = (dist_along_ray[j] <= m_laser_calib_params.intrinsics.max_range);
	    bool condn3 = (perp_dist[j] <= m_max_perp_dist_for_hit);
	    bool condn4 = (dist_along_ray[j] < min_dist_along_ray);
	    bool condn_all = condn1 && condn2 && condn3 && condn4;
	    if (condn_all)
	    {
		min_dist_along_ray = dist_along_ray[j];
		closest_id = j;
	    }
	}
	
	if (closest_id == -1)
	    hit_flag[i] = 0;
	else
	{
	    hit_flag[i] = 1;

	    // todo: add variance?
	    sim_pts[i] = m_pts[closest_id];
	}
    }

    return std::make_tuple(sim_pts, hit_flag);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
NearestNeighborSim::simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses)
{
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    for(size_t i = 0; i < imu_poses.size(); ++i)
    {
	std::vector<std::vector<double> > this_sim_pts;
	std::vector<int> this_hit_flag;
	std::tie(this_sim_pts, this_hit_flag) = simPtsGivenPose(imu_poses[i]); 
	for(size_t j = 0; j < this_sim_pts.size(); ++j)
	{
	    sim_pts.push_back(this_sim_pts[j]);
	    hit_flag.push_back(this_hit_flag[j]);
	}
    }

    return std::make_tuple(sim_pts, hit_flag);
}
