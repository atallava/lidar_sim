#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "dataanalysis.h"

#include <lidar_sim/DataProcessingUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    // pts from xyz
    std::string rel_path_xyz = "data/rim_stretch_veg_train.asc";
    std::vector<std::vector<double> > pts_stl = loadPtsFromXYZFile(rel_path_xyz);
    alglib::real_2d_array pts = convertStlPtsToAlglibPts(pts_stl);

    alglib::clusterizerstate clusterizer_state;
    alglib::ahcreport ahc_report;
    alglib::integer_1d_array cluster_idx;
    alglib::integer_1d_array cz;

    // run hierarchical clustering
    alglib::clusterizercreate(clusterizer_state);
    alglib::clusterizersetpoints(clusterizer_state, pts, 2);
    alglib::clusterizerrunahc(clusterizer_state, ahc_report);

    // get clusters
    int n_clusters = 2300;
    alglib::clusterizergetkclusters(ahc_report, n_clusters, cluster_idx, cz);

    // // with K=5, every points is assigned to its own cluster:
    // // C0=P0, C1=P1 and so on...
    // alglib::clusterizergetkclusters(rep, 5, cidx, cz);
    // printf("%s\n", cidx.tostring().c_str()); // EXPECTED: [0,1,2,3,4]

    // // with K=1 we have one large cluster C0=[P0,P1,P2,P3,P4,P5]
    // alglib::clusterizergetkclusters(rep, 1, cidx, cz);
    // printf("%s\n", cidx.tostring().c_str()); // EXPECTED: [0,0,0,0,0]

    // // with K=3 we have three clusters C0=[P3], C1=[P2,P4], C2=[P0,P1]
    // alglib::clusterizergetkclusters(rep, 3, cidx, cz);
    // printf("%s\n", cidx.tostring().c_str()); // EXPECTED: [2,2,1,0,1]

    return 0;
}
