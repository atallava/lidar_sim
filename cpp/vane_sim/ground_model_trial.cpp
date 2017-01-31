#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctime>

// alglib stuff
#include "stdafx.h"
#include "dataanalysis.h"

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

    // fit gp
    GroundModeler modeler;
    modeler.m_pts = pts;
    modeler.fitSmoothedPts();

    // debug
    RangeDataVizer vizer;
    // vizer.vizComparePts(pts, modeler.m_fit_pts);
    // vizer.vizPts(modeler.m_fit_pts);

    // delaunay triangulate
    std::cout << "triangulating: " << std::endl;
    modeler.delaunayTriangulate();

    // write out

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
