#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
 
#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/EllipsoidModeler.h>
#include <lidar_sim/RangeDataVizer.h>

using namespace lidar_sim;

std::string genRelPathPrimitivePts(int section_id, std::string class_name, int element_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/primitives/pts/" << class_name << "_" << element_id << ".asc";

    return ss.str();
}

std::string genRelPathPrimitiveEllipsoids(int section_id, std::string class_name, int element_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/primitives/ellipsoids/" << class_name << "_" << element_id << ".txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 3;
    std::string primitive_class = "medium_tree";
    int element_id = 1;

    std::vector<std::vector<double> > primitive_pts =
	loadPtsFromXYZFile(genRelPathPrimitivePts(section_id, primitive_class, element_id));
    std::string rel_path_ellipsoids = 
	genRelPathPrimitiveEllipsoids(section_id, primitive_class, element_id);
    EllipsoidModels ellipsoid_models = loadEllipsoidModelsFromFile(rel_path_ellipsoids);
	    
    // viz
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;
    actors.push_back(vizer.m_points_actor_server.genPointsActor(primitive_pts));
    std::vector<vtkSmartPointer<vtkActor> > ellipsoid_actors = 
	vizer.genEllipsoidModelsActors(ellipsoid_models);
    // actors.insert(actors.end(), ellipsoid_actors.begin(), ellipsoid_actors.end());

    vizer.takeItAway(actors);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}



