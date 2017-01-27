#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

#include "stdafx.h"
#include "dataanalysis.h"

#include <lidar_sim/EllipsoidVtkActorServer.h>
#include <lidar_sim/PointsVtkActorServer.h>
#include <lidar_sim/EllipsoidsModelUtils.h>
#include <lidar_sim/MathUtils.h>

namespace lidar_sim {
    std::vector<int> convertAlglib1DIntArrayToStlVector(alglib::integer_1d_array vec);
    std::vector<int> getNumPtsPerCluster(std::vector<int> pt_cluster_ids, int n_clusters);
    std::vector<int> getNumPtsPerCluster(alglib::integer_1d_array pt_cluster_ids, int n_clusters);
    void writeClusterIdsToFile(alglib::integer_1d_array pt_cluster_ids, std::string rel_path_output);
}


