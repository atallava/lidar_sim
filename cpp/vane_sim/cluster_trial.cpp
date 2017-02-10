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
#include <lidar_sim/EllipsoidModeler.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // pts from xyz
    std::string rel_path_xyz = "data/rim_stretch_veg_train.asc";
    // std::string rel_path_xyz = "data/sections/section_03/section_03_block_01_non_ground.xyz";
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
    size_t n_clusters = 250;
    alglib::clusterizergetkclusters(ahc_report, n_clusters, pt_cluster_ids, cz);

    // todo: delete.
    std::cout << "pt cluster ids: " << std::endl;
    for(size_t i = 0; i < 10; ++i)
    	std::cout << pt_cluster_ids[i] << " " ;
    std::cout << std::endl;

    // write clustering to file
    // std::string rel_path_output = "data/clustering.txt";
    // writeClusterIdsToFile(pt_cluster_ids, rel_path_output);
 
    // retain those with min pts
    std::vector<int> n_pts_per_cluster = getNumPtsPerCluster(pt_cluster_ids, n_clusters);
    
    // todo: delete
    std::cout << "n pts per cluster: " << std::endl;
    for(size_t i = 0; i < 10; ++i)
    	std::cout << n_pts_per_cluster[i] << " " ;
    std::cout << std::endl;
    exit(0); 

    int min_pts_per_cluster = 9;
    std::vector<int> selected_cluster_ids;
    for(size_t i = 0; i < n_pts_per_cluster.size(); ++i)
    	if (n_pts_per_cluster[i] >= min_pts_per_cluster)
    	    selected_cluster_ids.push_back(i);

    // ellipsoid models from selected clusters
    std::cout << "creating ellipsoid models..." << std::endl;
    int n_pts_in_clusters = 0;
    EllipsoidModels ellipsoid_models;
    for(size_t i = 0; i < selected_cluster_ids.size(); ++i)
    {
    	int this_cluster_id = selected_cluster_ids[i];
    	// get pts corresponding to this cluster
    	std::vector<std::vector<double> > this_cluster_pts;
    	for(size_t j = 0; j < n_pts; ++j)
    	    if (pt_cluster_ids[j] == this_cluster_id)
    	    {
    		this_cluster_pts.push_back(pts[j]);
    		n_pts_in_clusters++;
    	    }
	
    	ellipsoid_models.push_back(createEllipsoidModel(this_cluster_pts));
    }

    std::cout << "num clusters queried: " << n_clusters << std::endl;
    std::cout << "num selected clusters: " << selected_cluster_ids.size() << std::endl;
    std::cout << "fracn pts in clusters: " << n_pts_in_clusters/(double)pts.size() << std::endl;
    
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
