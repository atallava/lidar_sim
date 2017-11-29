#include <tuple>
#include <iomanip>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // load ellipsoids
    std::string rel_path_ellipsoid_models = "data/ellipsoid_models.txt";
    EllipsoidModels ellipsoid_models = 
	loadEllipsoidModelsFromFile(rel_path_ellipsoid_models);

    // load points
    std::string rel_path_xyz = "data/rim_stretch_veg_train.asc";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_xyz);

    const LaserCalibParams laser_calib_params;

    // specify pose
    std::vector<double> imu_pose = {455.729, -498.582, -5.825, 0.0397752, 0.022933, 5.07743};
    // std::vector<double> imu_pose = {450, -475, 0, 0, 0, deg2rad(10)};
    Eigen::MatrixXd T_imu_world = getImuTransfFromPose(imu_pose);

    std::vector<double> ray_origin = {imu_pose[1], imu_pose[0], imu_pose[2]};
    // single ray dirn
    // std::vector<std::vector<double> > ray_dirns = {{-0.146803, -0.982715, -0.112786}};
    // scanning pattern
    std::vector<std::vector<double> > ray_dirns = genRayDirnsWorldFrame(imu_pose, laser_calib_params);

    // sim object
    EllipsoidModelSim sim;
    sim.setEllipsoidModels(ellipsoid_models);
    sim.setLaserCalibParams(laser_calib_params);

    // ellipsoid intersections
    std::cout << "calculating ellipsoid intersections..." << std::endl;
    std::vector<std::vector<int> > intersection_flag;
    std::vector<std::vector<double> > dist_along_ray;
    std::tie(intersection_flag, dist_along_ray) = sim.calcEllipsoidIntersections(
    	ray_origin, ray_dirns);
    std::vector<int> intersected_ellipsoids_flag = 
	getIntersectedFlag(intersection_flag);   

    std::cout << "simulating from ellipsoids..." << std::endl;
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    std::tie(sim_pts, hit_flag) = sim.simPtsGivenIntersections(intersection_flag, dist_along_ray);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << std::setprecision(15) << "elapsed time: " << elapsed_time << "s" << std::endl;

    return(1);
}
