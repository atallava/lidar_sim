#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <boost/program_options.hpp>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
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

using namespace lidar_sim;

std::string genRelPathBlock(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

    return ss.str();
}

std::string genRelPathEllipsoids(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

std::string genRelPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

    return ss.str();
}

std::string genRelPathBlockNodeIdsNonGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/block_node_ids_non_ground.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // parse command line inputs
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
	("help", "Print help messages")
	("section_id", po::value<int>(), "Section id")
	("block_id", po::value<int>(), "Block id")
	("rel_path_ellipsoids", po::value<std::string>(), "Where ellipsoids are stored")
	("n_clusters_per_pt", po::value<double>(), 
	 "Num clusters per point. Parameter in EllipsoidModeler.")
	("max_maha_dist_for_hit", po::value<double>(), 
	 "Max mahalanobis distance for hit. Parameter in EllipsoidModelSim.")
	("verbose", po::value<int>(), "Print info");
    
    po::variables_map vm;

    int section_id, block_id;
    std::string rel_path_ellipsoids;
    int verbose;
    bool set_n_clusters_per_pt = false;
    double n_clusters_per_pt = 0.001;
    bool set_max_maha_dist_for_hit = false;
    double max_maha_dist_for_hit = 3.5;
    try
    {
	po::store(po::parse_command_line(argc, argv, desc), vm);
	
	if (vm.count("help"))
	    std::cout << desc << std::endl;

	if (vm.count("section_id") && vm.count("block_id"))
	{
	    section_id = vm["section_id"].as<int>();
	    block_id = vm["block_id"].as<int>();
	}
	else
	{
	    bool condn1 = vm.count("section_id") && ~vm.count("block_id");
	    bool condn2 = ~vm.count("section_id") && vm.count("block_id");
	    if (condn1 || condn2) {
		std::stringstream ss_err_msg;
		ss_err_msg << "cannot input only block or section id";
		throw std::runtime_error(ss_err_msg.str().c_str());
	    }
	    else {
		section_id = 3;
		block_id = 8;
	    }
	}

	if (vm.count("rel_path_ellipsoids"))
	    rel_path_ellipsoids = vm["rel_path_ellipsoids"].as<std::string>();
	else
	    rel_path_ellipsoids = genRelPathEllipsoids(section_id, block_id);

	if (vm.count("n_clusters_per_pt"))
	{
	    set_n_clusters_per_pt = true;
	    n_clusters_per_pt = vm["n_clusters_per_pt"].as<double>();
	}

	if (vm.count("max_maha_dist_for_hit"))
	{
	    set_max_maha_dist_for_hit = true;
	    max_maha_dist_for_hit = vm["max_maha_dist_for_hit"].as<double>();
	}

	if (vm.count("verbose"))
	    verbose = vm["verbose"].as<int>();
	else
	    verbose = 0;

	po::notify(vm);
    }
    catch(po::error& e) 
    { 
	std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
	std::cerr << desc << std::endl; 
	return 1;
    } 

    if (verbose)
	std::cout << "section id: " << section_id << ", block_id: " << block_id << "..." << std::endl;

    std::string rel_path_pts = genRelPathBlock(section_id, block_id);
    std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);
	
    EllipsoidModeler modeler;
    modeler.setDebugFlag(verbose);
    // setting options for sim optim
    if (set_n_clusters_per_pt)
	modeler.m_n_clusters_per_pt = n_clusters_per_pt;
    if (set_max_maha_dist_for_hit)
    {
	modeler.m_set_max_maha_dist_for_hit = true;
	modeler.m_max_maha_dist_for_hit = max_maha_dist_for_hit;
    }
    else
	modeler.m_set_max_maha_dist_for_hit = false;

    modeler.createEllipsoidModels(rel_path_pts);

    // prob hit calc
    int calc_hit_prob = 1;
    if (calc_hit_prob) 
    {
	// section
	std::string rel_path_section = "data/section_03_world_frame_subsampled.xyz";
	SectionLoader section(rel_path_section);

	// pose server
	std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
	PoseServer imu_pose_server(rel_path_poses_log);

	// blocks info
	std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_id);
	std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_id);
	std::vector<std::vector<double> > imu_posn_nodes = loadArray(genRelPathImuPosnNodes(section_id), 3);
	std::vector<std::vector<int> > block_node_ids_non_ground = 
	    doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));

	// calc section pts to process
	int num_nbrs = 1;
	std::vector<std::vector<int> > section_nbr_pt_ids; 

	std::tie(section_nbr_pt_ids, std::ignore) = nearestNeighbors(section.m_pts, block_pts, num_nbrs);

	std::vector<std::vector<double> > section_pts_to_process;
	std::vector<int> section_pt_ids_to_process;
	for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	    for(size_t j = 0; j < section_nbr_pt_ids[i].size(); ++j)
	    {
		int id = section_nbr_pt_ids[i][j];
		section_pt_ids_to_process.push_back(id);
		section_pts_to_process.push_back(section.m_pts[id]);
	    }
	modeler.calcHitProb(section, section_pt_ids_to_process, imu_pose_server);
    }
    else
	if (verbose)
	    std::cout << "skipping hit prob calc..." << std::endl;

    // write out
    // rel_path_ellipsoids = genRelPathEllipsoids(section_id, block_id);
    // rel_path_ellipsoids = "data/hg_optim/ellipsoids.txt";

    modeler.writeEllipsoidsToFile(rel_path_ellipsoids);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    if (verbose)
	std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

