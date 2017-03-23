#include <tuple>

#include <vtkProperty.h>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
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

    // viz
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;
    bool use_hit_flag = true;

    // debug
    // std::cout << "hit flag: " << std::endl;
    // dispVec(findNonzeroIds(hit_flag));

    // triangles
    actors.push_back(
	vizer.m_triangles_actor_server.genTrianglesActor(sim.m_triangles, sim.m_fit_pts));

    // ray
    for(size_t i = 0; i < ray_dirns.size(); ++i)
    {
	if (use_hit_flag)
	    if (!hit_flag[i])
		continue;
	actors.push_back(
	    vizer.m_line_actor_server.genLineActorDirn(ray_origin, ray_dirns[i]));
    }

    // pts
    std::vector<std::vector<double> > sim_pts_to_plot;
    if (use_hit_flag)
    	sim_pts_to_plot = logicalSubsetArray(sim_pts, hit_flag);
    else
    	sim_pts_to_plot = sim_pts;
    vtkSmartPointer<vtkActor> pts_actor = vizer.m_points_actor_server.genPointsActor(sim_pts_to_plot);
    pts_actor->GetProperty()->SetColor(0, 1, 0);
    actors.push_back(pts_actor);

    // fire up
    vizer.takeItAway(actors);

    return(1);
}
