#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctime>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/ClusteringUtils.h>
#include <lidar_sim/TriangleModeler.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // pts from xyz
    // std::string rel_path_xyz = "data/rim_stretch_ground_train.asc";
    std::string rel_path_pts = "data/block_trial.asc";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // fit smoothed pts
    TriangleModeler modeler;
    modeler.setDebugFlag(1);
    modeler.createTriangleModels(rel_path_pts);

    // write out
    std::string rel_path_triangles = "data/triangle_models.txt";
    modeler.writeTrianglesToFile(rel_path_triangles);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
