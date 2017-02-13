#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/SceneObjectServer.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    std::string rel_path_annotations = "data/sections/section_04/section_04_annotations.txt";
    std::vector<std::string> object_classes;
    std::vector<std::vector<double> > object_posns;
    std::tie(object_classes, object_posns) = 
	loadAnnotations(rel_path_annotations);

    // the object server has to know where the primitives live
    // it will load them all into its memory
    // when queries, it will select one of the appropriate type
    // and transform it
    SceneObjectServer object_server;
    std::string rel_path_primitive_ellipsoids_dir = "data/sections/section_03/primitives/ellipsoids";
    object_server.loadPrimitiveEllipsoidModels(rel_path_primitive_ellipsoids_dir);

    // EllipsoidModels ellipsoid_models;
    // for(size_t i = 0; i < object_classes.size(); ++i)
    // {
    // 	// todo: currently not taking into account orientation of primitive
    // 	EllipsoidModels.push_back(
    // 	    object_server.genObjectAtPosn(object_classes[i], object_posns[i]));
    // }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}


// load the annotation file

// transform the ellipsoids

// associate them with nodes

// create blocks

// write out



