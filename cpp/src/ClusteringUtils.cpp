#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <vtkSmartPointer.h>
#include <vtkCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkTextMapper.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkProperty.h>
#include <vtkTextProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkParametricFunctionSource.h>
#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkParametricEllipsoid.h>
#include <vtkPropAssembly.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/ClusteringUtils.h>
#include <lidar_sim/VizUtils.h>

namespace lidar_sim {
    std::vector<int> convertAlglib1DIntArrayToStlVector(alglib::integer_1d_array vec)
    {
	std::vector<int> vec_stl;
	
	for(size_t i = 0; i < vec.length(); ++i)
	    vec_stl.push_back((int)vec[i]);

	return vec_stl;
    }

    std::vector<int> getNumPtsPerCluster(alglib::integer_1d_array pt_cluster_ids, int n_clusters)
    {
	std::vector<int> ids = convertAlglib1DIntArrayToStlVector(pt_cluster_ids);
	return getNumPtsPerCluster(ids, n_clusters);
    }

    std::vector<int> getNumPtsPerCluster(std::vector<int> pt_cluster_ids, int n_clusters)
    {
	std::vector<int> n_pts_per_cluster(n_clusters, 0);
    
	for(size_t i = 0; i < n_clusters; ++i)
	    for(size_t j = 0; j < pt_cluster_ids.size(); ++j)
		if (pt_cluster_ids[j] == i)
		    n_pts_per_cluster[i]++;

	return n_pts_per_cluster;
    }

    void writeClusterIdsToFile(alglib::integer_1d_array pt_cluster_ids, std::string rel_path_output)
    {
	std::ofstream file(rel_path_output);
	std::cout << "Writing cluster ids to: " << rel_path_output << std::endl;
	
	for(size_t i = 0; i < pt_cluster_ids.length(); ++i)
	    file << pt_cluster_ids[i] << std::endl;

	file.close();
    }
}
