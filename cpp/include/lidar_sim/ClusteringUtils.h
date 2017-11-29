#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

#include "stdafx.h"
#include "dataanalysis.h"

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>

namespace lidar_sim {
    std::vector<int> getNumPtsPerCluster(std::vector<int> pt_cluster_ids, int n_clusters);
    std::vector<int> getNumPtsPerCluster(alglib::integer_1d_array pt_cluster_ids, int n_clusters);
    void writeClusterIdsToFile(alglib::integer_1d_array pt_cluster_ids, std::string rel_path_output);
}


