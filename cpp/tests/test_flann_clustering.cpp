#include <ctime>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/EllipsoidModeler.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    // load a block
    int section_id = 3;
    int block_id = 10;
    std::string path_block_pts = genPathNonGroundBlockPts(section_id, block_id);

    int verbose = 1;
    clock_t start_time = clock();

    // flann ellipsoids
    EllipsoidModeler modeler_flann;
    modeler_flann.setVerbosity(verbose);
    modeler_flann.createEllipsoidModels(path_block_pts);
    std::string rel_path_ellipsoids_flann = "data/misc/ellipsoids_flann.txt";
    modeler_flann.writeEllipsoidsToFile(rel_path_ellipsoids_flann);
    double flann_time = (clock()-start_time)/CLOCKS_PER_SEC;
    if (verbose)
	std::cout << "flann time : " << flann_time << "s." << std::endl;

    // alglib ellipsoids
    // EllipsoidModeler modeler_alglib;
    // modeler_alglib.setVerbosity(verbose);
    // modeler_alglib.createEllipsoidModels(path_block_pts);
    // std::string rel_path_ellipsoids_alglib = "data/misc/ellipsoids_alglib.txt";
    // modeler_alglib.writeEllipsoidsToFile(rel_path_ellipsoids_alglib);
    // double alglib_time = (clock()-start_time)/CLOCKS_PER_SEC - flann_time;
    // if (verbose)
    // 	std::cout << "alglib time : " << alglib_time << "s." << std::endl;

    return(1);
}
