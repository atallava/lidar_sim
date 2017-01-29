#include <tuple>
#include <ctime>

#include <vtkProperty.h>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    // load pts1
    std::string rel_path_pts1 = "data/rim_stretch_veg_validation.asc";
    std::vector<std::vector<double> > pts1 = loadPtsFromXYZFile(rel_path_pts1);

    // load pts2
    std::string rel_path_pts2 = "data/rim_stretch_veg_validation_sim.xyz";
    // std::string rel_path_pts2 = "data/rim_stretch_veg_train.asc";
    std::vector<std::vector<double> > pts2 = loadPtsFromXYZFile(rel_path_pts2);

    // viz
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;
    
    std::vector<double> color1 = {1, 1, 1};
    vtkSmartPointer<vtkActor> actor1 = vizer.m_points_actor_server.genPointsActor(pts1);
    actor1->GetProperty()->SetColor(color1[0], color1[1], color1[2]);
    actors.push_back(actor1);

    std::vector<double> color2 = {1, 0, 0};
    vtkSmartPointer<vtkActor> actor2 = vizer.m_points_actor_server.genPointsActor(pts2);
    actor2->GetProperty()->SetColor(color2[0], color2[1], color2[2]);
    actors.push_back(actor2);

    // fire up
    vizer.takeItAway(actors);

    return(1);
}
