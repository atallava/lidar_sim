#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctime>

#include <vtkProperty.h>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/ClusteringUtils.h>
#include <lidar_sim/GroundModeler.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // pts from xyz
    std::string rel_path_xyz = "data/rim_stretch_ground_train.asc";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_xyz);

    // fit smoothed pts
    GroundModeler modeler;
    modeler.m_pts = pts;
    modeler.fitSmoothedPts();

    // delaunay triangulate
    std::cout << "triangulating " << std::endl;
    modeler.delaunayTriangulate();

    // viz
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;
    vtkSmartPointer<vtkActor> pts_actor = 
	vizer.m_points_actor_server.genPointsActor(pts);
    pts_actor->GetProperty()->SetColor(1, 0, 0);
    actors.push_back(pts_actor);
    actors.push_back(
	vizer.m_triangles_actor_server.genTrianglesActor(modeler.m_triangles, modeler.m_fit_pts));
    vizer.takeItAway(actors);

    // write out
    // std::string rel_path_triangles = "data/triangle_models.txt";
    // modeler.writeTrianglesToFile(rel_path_triangles);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
