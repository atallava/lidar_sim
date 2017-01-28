#include <tuple>

#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    // load ellipsoids
    std::string rel_path_ellipsoid_models = "data/ellipsoid_models.txt";
    EllipsoidModels ellipsoid_models = 
	loadEllipsoidModels(rel_path_ellipsoid_models);

    // load points
    std::string rel_path_xyz = "data/rim_stretch_veg_train.asc";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_xyz);

    const LaserCalibParams laser_calib_params;

    // specify pose
    std::vector<double> imu_pose = {450, -475, 0, 0, 0, deg2rad(10)};
    Eigen::MatrixXd T_imu_world = getImuTransfFromPose(imu_pose);

    std::vector<double> ray_origin = {imu_pose[1], imu_pose[0], imu_pose[2]};
    std::vector<std::vector<double> > ray_dirns = {{-0.1464, -0.9798, 0.1360}};

    // sim object
    EllipsoidModelSim sim;
    sim.setEllipsoidModels(ellipsoid_models);
    sim.setLaserCalibParams(laser_calib_params);

    std::vector<std::vector<int> > intersection_flag;
    std::vector<std::vector<double> > dist_along_ray;
    std::tie(intersection_flag, dist_along_ray) = sim.calcEllipsoidIntersections(
    	ray_origin, ray_dirns);

    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    std::tie(sim_pts, hit_flag) = sim.simPtsGivenIntersections(intersection_flag, dist_along_ray);

    // viz
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;
    // ellipsoids
    for(size_t i = 0; i < ellipsoid_models.size(); ++i)
    	actors.push_back(
    	    vizer.m_ellipsoid_actor_server.genEllipsoidActor(ellipsoid_models[i].mu, ellipsoid_models[i].cov_mat));
    // ray
    actors.push_back(
    	vizer.m_line_actor_server.genLineActorDirn(ray_origin, ray_dirns[0]));
    // pts
    actors.push_back(
	vizer.m_points_actor_server.genPointsActor(sim_pts));

    // fire up
    vizer.takeItAway(actors);

    return(1);
}
