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

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // pts
    std::string rel_path_pts = "data/sections/section_03/primitives/pts/large_tree_1.asc";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);
    std::vector<std::vector<double> > centered_pts = centerPts(pts);

    // obb
    OrientedBox obb = calcObb(centered_pts);
    obb.dispBox();
    obb.calcVertices();
    
    // viz
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;
    actors.push_back(vizer.m_points_actor_server.genPointsActor(centered_pts));
    actors.push_back(vizer.genPolyActor(obb.m_vertices));

    // write fig
    std::string rel_path_fig = "figs/screenshot2.png";
    vizer.writeActorsToFile(actors, rel_path_fig);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}



