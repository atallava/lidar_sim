#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// alglib stuff
#include "stdafx.h"
#include "dataanalysis.h"

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/ClusteringUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // pts from xyz
    // std::string rel_path_xyz = "data/rim_stretch_veg_train.asc";
    std::string rel_path_xyz = "data/dummy_cluster.xyz";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_xyz);
    size_t n_pts = pts.size();
    alglib::real_2d_array pts_alglib = convertStlPtsToAlglibPts(pts);

    alglib::clusterizerstate clusterizer_state;
    alglib::ahcreport ahc_report;
    alglib::integer_1d_array pt_cluster_ids;
    alglib::integer_1d_array cz;

    // run hierarchical clustering
    std::cout << "clustering..." << std::endl;
    alglib::clusterizercreate(clusterizer_state);
    alglib::clusterizersetpoints(clusterizer_state, pts_alglib, 2);
    alglib::clusterizerrunahc(clusterizer_state, ahc_report);

    // get clusters
    // size_t n_clusters = 2300;
    size_t n_clusters = 1;
    alglib::clusterizergetkclusters(ahc_report, n_clusters, pt_cluster_ids, cz);

    // write clustering to file
    // std::string rel_path_output = "data/clustering.txt";
    // writeClusterIdsToFile(pt_cluster_ids, rel_path_output);
 
    // retain those with min pts
    std::vector<int> n_pts_per_cluster = getNumPtsPerCluster(pt_cluster_ids, n_clusters);
    int min_pts_per_cluster = 9;
    std::vector<int> selected_cluster_ids;
    for(size_t i = 0; i < n_pts_per_cluster.size(); ++i)
	if (n_pts_per_cluster[i] >= min_pts_per_cluster)
	    selected_cluster_ids.push_back(i);

    std::cout << "num selected clusters: " << selected_cluster_ids.size() << std::endl;
    
    // ellipsoid models from selected clusters
    std::cout << "creating ellipsoid models..." << std::endl;
    EllipsoidModels ellipsoid_models;
    for(size_t i = 0; i < selected_cluster_ids.size(); ++i)
    {
	int this_cluster_id = selected_cluster_ids[i];
	// get pts corresponding to this cluster
	std::vector<std::vector<double> > this_cluster_pts;
	for(size_t j = 0; j < n_pts; ++j)
	    if (pt_cluster_ids[j] == this_cluster_id)
		this_cluster_pts.push_back(pts[j]);
	
	ellipsoid_models.push_back(createEllipsoidModel(this_cluster_pts));
    }

    // write to file
    // std::string rel_path_ellipsoid_models = "data/ellipsoid_models.txt";
    // writeEllipsoidModelsToFile(ellipsoid_models, rel_path_ellipsoid_models);

    
    // viz
    RangeDataVizer vizer;
    std::cout << "vizing ellipsoid models..." << std::endl;
    vizer.vizEllipsoidModels(ellipsoid_models, pts);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << std::setprecision(15) << "elapsed time: " << elapsed_time << "s" << std::endl;

    return(1);
}
