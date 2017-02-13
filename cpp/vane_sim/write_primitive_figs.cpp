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
#include <lidar_sim/EllipsoidModelUtils.h>
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

std::string genRelPathPrimitivePts(int section_id, std::string class_name, int element_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/primitives/pts/" << class_name << "_" << element_id << ".asc";

    return ss.str();
}

std::string genRelPathPrimitiveEllipsoids(int section_id, std::string class_name, int element_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/primitives/ellipsoids/" << class_name << "_" << element_id << ".txt";

    return ss.str();
}

std::string genRelPathPrimitiveFig(int section_id, std::string class_name, int element_id)
{
    std::ostringstream ss;
    ss << "figs/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/primitives/" << class_name << "_" << element_id << ".png";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // class names
    std::vector<std::string> primitive_classes {"low_shrub", "low_shrub_patch", "medium_shrub", "medium_shrub_patch", "thin_shrub", "large_shrub", "large_shrub_patch",
	    "medium_tree", "large_tree"};

    int section_id = 3;

    // element ids
    // todo: automate this
    std::vector<std::vector<int> > element_ids;
    {
	std::vector<int> low_shrub_ids;
	for(size_t i = 1; i <= 11; ++i)
	    low_shrub_ids.push_back(i);
	element_ids.push_back(low_shrub_ids);

	std::vector<int> low_shrub_patch_ids;
	for(size_t i = 1; i <= 12; ++i)
	    low_shrub_patch_ids.push_back(i);
	element_ids.push_back(low_shrub_patch_ids);

	std::vector<int> medium_shrub_ids;
	for(size_t i = 1; i <= 21; ++i)
	{
	    if (i == 9)
		continue;
	    medium_shrub_ids.push_back(i);
	}
	element_ids.push_back(medium_shrub_ids);

	std::vector<int> medium_shrub_patch_ids;
	for(size_t i = 1; i <= 11; ++i)
	    medium_shrub_patch_ids.push_back(i);
	element_ids.push_back(medium_shrub_patch_ids);

	std::vector<int> thin_shrub_ids;
	for(size_t i = 1; i <= 2; ++i)
	    thin_shrub_ids.push_back(i);
	element_ids.push_back(thin_shrub_ids);

	std::vector<int> large_shrub_ids;
	for(size_t i = 1; i <= 4; ++i)
	    large_shrub_ids.push_back(i);
	element_ids.push_back(large_shrub_ids);

	std::vector<int> large_shrub_patch_ids;
	for(size_t i = 1; i <= 4; ++i)
	    large_shrub_patch_ids.push_back(i);
	element_ids.push_back(large_shrub_patch_ids);

	std::vector<int> medium_tree_ids;
	for(size_t i = 1; i <= 22; ++i)
	    medium_tree_ids.push_back(i);
	element_ids.push_back(medium_tree_ids);

	std::vector<int> large_tree_ids;
	for(size_t i = 1; i <= 11; ++i)
	    large_tree_ids.push_back(i);
	element_ids.push_back(large_tree_ids);
    } 

    RangeDataVizer vizer;
    
    for(size_t i = 0; i < primitive_classes.size(); ++i)
	for(size_t j = 0; j < element_ids[i].size(); ++j)
	{
	    std::vector<std::vector<double> > primitive_pts =
		loadPtsFromXYZFile(genRelPathPrimitivePts(section_id, primitive_classes[i], element_ids[i][j]));
	    std::string rel_path_ellipsoids = 
		genRelPathPrimitiveEllipsoids(section_id, primitive_classes[i], element_ids[i][j]);
	    EllipsoidModels ellipsoid_models = loadEllipsoidModelsFromFile(rel_path_ellipsoids);
	    
	    // gen actors
	    std::vector<vtkSmartPointer<vtkActor> > actors;
	    actors.push_back(vizer.m_points_actor_server.genPointsActor(primitive_pts));
	    std::vector<vtkSmartPointer<vtkActor> > ellipsoid_actors = 
		vizer.genEllipsoidModelsActors(ellipsoid_models);
	    actors.insert(actors.end(), ellipsoid_actors.begin(), ellipsoid_actors.end());

	    // write
	    std::string rel_path_fig = genRelPathPrimitiveFig(section_id, primitive_classes[i], element_ids[i][j]);
	    vizer.writeActorsToFile(actors, rel_path_fig);
	}

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}



