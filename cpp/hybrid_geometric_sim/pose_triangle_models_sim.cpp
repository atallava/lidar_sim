#include <tuple>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    // sim object
    std::string rel_path_triangle_models = "data/triangle_models.txt";
    TriangleModelSim sim;
    const LaserCalibParams laser_calib_params;
    sim.loadTriangleModels(rel_path_triangle_models);
    sim.setLaserCalibParams(laser_calib_params);
    
    // specify pose
    std::vector<double> imu_pose = {440, -460, 0, 0.0, 0.0, deg2rad(10)};
    Eigen::MatrixXd T_imu_world = getImuTransfFromPose(imu_pose);

    std::vector<double> ray_origin = {imu_pose[1], imu_pose[0], imu_pose[2]};
    // single ray dirn
    // std::vector<std::vector<double> > ray_dirns = {{std::cos(deg2rad(50)), 0, -std::sin(deg2rad(50))}};
    // scanning pattern
    std::vector<std::vector<double> > ray_dirns = genRayDirnsWorldFrame(imu_pose, laser_calib_params);

    // tri intersections
    std::cout << "calculating tri intersections..." << std::endl;
    std::vector<std::vector<int> > intersection_flag;
    std::vector<std::vector<double> > dist_along_ray;
    std::tie(intersection_flag, dist_along_ray) = sim.calcTriIntersections(ray_origin, ray_dirns);

    std::vector<int> intersected_tri_flag = 
	getIntersectedFlag(intersection_flag);   

    std::cout << "simulating from tri..." << std::endl;
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    std::tie(sim_pts, hit_flag) = sim.simPtsGivenIntersections(ray_origin, ray_dirns, 
    							       intersection_flag, dist_along_ray);

    return(1);
}
