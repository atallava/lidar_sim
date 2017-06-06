#include <iostream>
#include <sstream>

#include <lidar_sim/ModelingUtils.h>

using namespace lidar_sim;

namespace lidar_sim {
    EllipsoidModel createEllipsoidModel(Pts pts)
    {
	EllipsoidModel model;
	model.mu = calcPtsMean(pts);
	model.cov_mat = calcPtsCovMat(pts);
	model.hit_prob = 1;

	return model;
    }

    void writeEllipsoidModelsToFile(EllipsoidModels ellipsoid_models, std::string rel_path_output)
    {
	std::ofstream file(rel_path_output);
	std::cout << "Writing ellipsoid models to: " << rel_path_output << std::endl;
	
	for(size_t i = 0; i < ellipsoid_models.size(); ++i)
	{
	    // mu
	    std::vector<double> mu = ellipsoid_models[i].mu;
	    std::ostringstream line;
	    for(size_t j = 0; j < 3; ++j)
		line << mu[j] << " ";

	    // cov mat
	    Eigen::MatrixXd cov_mat = ellipsoid_models[i].cov_mat;
	    for(size_t j = 0; j < 3; ++j)
		for(size_t k = 0; k < 3; ++k)
		    line << cov_mat(k,j) << " ";
	    
	    // hit_prob
	    double hit_prob = ellipsoid_models[i].hit_prob;
	    line << hit_prob << std::endl;
	    
	    file << line.str();
	}

	file.close();
    }

    EllipsoidModels loadEllipsoidModelsFromFile(std::string rel_path_input)
    {
	// open input file
	std::ifstream file(rel_path_input);
	std::cout << "Reading ellipsoid models from: " << rel_path_input << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	EllipsoidModels ellipsoid_models;

	std::string current_line;
	while(std::getline(file, current_line))
	{
	    std::istringstream iss(current_line);

	    std::vector<double> mu(3,0);
	    for(size_t i = 0; i < 3; ++i)
		iss >> mu[i];

	    Eigen::MatrixXd cov_mat(3,3);
	    for(size_t i = 0; i < 3; ++i)
		for(size_t j = 0; j < 3; ++j)
		    iss >> cov_mat(j,i);

	    double hit_prob;
	    iss >> hit_prob;

	    EllipsoidModel ellipsoid_model;
	    ellipsoid_model.mu = mu;
	    ellipsoid_model.cov_mat = cov_mat;
	    ellipsoid_model.hit_prob = hit_prob;

	    ellipsoid_models.push_back(ellipsoid_model);
	}

	file.close();

	return ellipsoid_models;
    }

    void dispEllipsoidModel(EllipsoidModel model)
    {
	std::cout << "mu: " << std::endl;
	for(size_t i = 0; i < 3; ++i)
	    std::cout << model.mu[i] << " ";
	std::cout << std::endl;

	std::cout << "cov_mat: " << std::endl;
	std::cout << model.cov_mat << std::endl;

	std::cout << "prob_hit: " << std::endl;
	std::cout << model.hit_prob << std::endl;
    }

    void writeTriangleModelsToFile(TriangleModels triangle_models, std::string rel_path_output)
    {
	std::ofstream file(rel_path_output);
	std::cout << "ModelingUtils: writing triangles to: " << rel_path_output << std::endl;

	file << "pts" << std::endl;
    
	for(size_t i = 0; i < triangle_models.m_fit_pts.size(); ++i)
	    file << triangle_models.m_fit_pts[i][0] << " " <<
		triangle_models.m_fit_pts[i][1] << " " <<
		triangle_models.m_fit_pts[i][2] << std::endl;

	file << "triangles" << std::endl;
	for(size_t i = 0; i < triangle_models.m_triangles.size(); ++i)
	    file << triangle_models.m_triangles[i][0] << " " <<
		triangle_models.m_triangles[i][1] << " " <<
		triangle_models.m_triangles[i][2] << " " << triangle_models.m_hit_prob_vec[i] << std::endl;
    
	file.close();
    }

    TriangleModels loadTriangleModelsFromFile(std::string rel_path_input)
    {
	// open input file
	std::ifstream file(rel_path_input);
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}
	std::cout << "Reading triangle models from: " << rel_path_input << std::endl;

	TriangleModels triangle_models;
	std::string current_line;
	std::string mode;
	while(std::getline(file, current_line))
	{
	    if (strcmp(current_line.c_str(), "pts") == 0)
	    {
		mode = "pts";
		continue;
	    }
	    if (strcmp(current_line.c_str(), "triangles") == 0)
	    {
		mode = "triangles";
		continue;
	    }
	
	    std::istringstream iss(current_line);

	    if (strcmp(mode.c_str(), "pts") == 0)
	    {
		std::vector<double> pt(3,0);
		for(size_t i = 0; i < 3; ++i)
		    iss >> pt[i];

		triangle_models.m_fit_pts.push_back(pt);
	    }

	    if (strcmp(mode.c_str(), "triangles") == 0)
	    {
		std::vector<int> triangle(3,0);
		for(size_t i = 0; i < 3; ++i)
		    iss >> triangle[i];

		triangle_models.m_triangles.push_back(triangle);

		double hit_prob;
		iss >> hit_prob;
		triangle_models.m_hit_prob_vec.push_back(hit_prob);
	    }
		    
	}
	file.close();

	return triangle_models;
    }

    TriangleModels loadMeshModelFromPly(std::string rel_path_input)
    {
	std::ifstream file(rel_path_input);
	std::cout << "Loading mesh models from: " << rel_path_input << std::endl;

	TriangleModels triangle_models;
	std::string current_line;
	bool reading_header = true;
	bool reading_vertices = false;
	int n_vertices;
	int vertex_read_count = 0;
	while(std::getline(file, current_line))
	{
	    if (reading_header)
	    {
		if (current_line.find("element vertex") != std::string::npos)
		{
		    std::istringstream iss(current_line);
		    std::string ignore_str;
		    iss >> ignore_str; iss >> ignore_str;
		    iss >> n_vertices;
		}
		if (current_line.find("end_header") != std::string::npos)
		{
		    reading_header = false;
		    reading_vertices = true;
		}
	    }
	    else
	    {
		if (reading_vertices)
		{
		    vertex_read_count++;
		    std::istringstream iss(current_line);
		    std::vector<double> pt(3,0);
		    for(size_t i = 0; i < 3; ++i)
			iss >> pt[i];
		    triangle_models.m_fit_pts.push_back(pt);

		    if (vertex_read_count >= n_vertices)
		    {
			reading_vertices = false;
		    }
		}
		else
		{
		    std::istringstream iss(current_line);
		    int throw_val;
		    iss >> throw_val;
		    std::vector<int> triangle(3,0);
		    for(size_t i = 0; i < 3; ++i)
			iss >> triangle[i];

		    triangle_models.m_triangles.push_back(triangle);
		}
	    }
	}
	file.close();

	// default hit prob
	double hit_prob_default = 1;
	std::vector<double> hit_prob_vec(triangle_models.m_triangles.size(), hit_prob_default);
	triangle_models.m_hit_prob_vec = hit_prob_vec;

	return triangle_models;
    }

    TriangleModels stitchTriangleModels(std::vector<TriangleModels> &triangle_models_vec)
    {
	TriangleModels triangle_models;
	for(size_t i = 0; i < triangle_models_vec.size(); ++i)
	{
	    TriangleModels this_triangle_models = triangle_models_vec[i];

	    std::vector<std::vector<int> > this_triangles = this_triangle_models.m_triangles;
	    int offset = triangle_models.m_fit_pts.size();
	    // since triangles are pt ids
	    for(size_t i = 0; i < this_triangles.size(); ++i)
		for(size_t j = 0 ; j < this_triangles[i].size(); ++j)
		    this_triangles[i][j] += offset;
	    triangle_models.m_triangles.insert(triangle_models.m_triangles.end(),
					       this_triangles.begin(), this_triangles.end());

	    triangle_models.m_fit_pts.insert(triangle_models.m_fit_pts.end(),
					     this_triangle_models.m_fit_pts.begin(), this_triangle_models.m_fit_pts.end());
	    triangle_models.m_hit_prob_vec.insert(triangle_models.m_hit_prob_vec.end(),
						  this_triangle_models.m_hit_prob_vec.begin(), this_triangle_models.m_hit_prob_vec.end());
						  
	}

	return triangle_models;
    }

    EllipsoidModels stitchEllipsoidModels(std::vector<EllipsoidModels> &ellipsoid_models_vec)
    {
	EllipsoidModels ellipsoid_models;
	for(size_t i = 0; i < ellipsoid_models_vec.size(); ++i)
	    ellipsoid_models.insert(ellipsoid_models.end(), 
				    ellipsoid_models_vec[i].begin(), ellipsoid_models_vec[i].end());

	return ellipsoid_models;
    }

    std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<int> > > 
    buildBlocks(const std::vector<std::vector<double> > &imu_posn_nodes,
		const std::vector<std::vector<double> > &pts, int pts_per_block)
    {
	// nearest neighbor for pts in nodes
	std::vector<std::vector<int> > nn_ids;
	std::tie(nn_ids, std::ignore) = nearestNeighbors(imu_posn_nodes, pts, 1);
    
	// for each node, which pts are in it
	std::vector<std::vector<int> > node_pts_map(imu_posn_nodes.size(), 
						    std::vector<int> ());
    
	for(size_t i = 0; i < pts.size(); ++i)
	    node_pts_map[nn_ids[i][0]].push_back(i);
    
	// block node ids
	std::vector<std::vector<int> > block_node_ids;
	std::vector<int> current_block_node_ids(2, 0);
	current_block_node_ids[0] = 0;

	int block_id = 1;

	int pts_in_current_block = 0;
	for(size_t i = 0; i < node_pts_map.size(); ++i)
	{
	    pts_in_current_block += node_pts_map[i].size();

	    if (pts_in_current_block >= pts_per_block)
	    {
		if (i == node_pts_map.size()-1)
		    break;

		// block node ids
		current_block_node_ids[1] = i;
		block_node_ids.push_back(current_block_node_ids);
		current_block_node_ids[0] = i+1;

		// increment block id
		block_id++;

		// reset counter
		pts_in_current_block = 0;
	    }
	}
	current_block_node_ids[1] = node_pts_map.size()-1;
	block_node_ids.push_back(current_block_node_ids);

	
	// pts ids in each block
	std::vector<std::vector<int> > block_pts_ids(block_node_ids.size(), std::vector<int>());
	for(size_t i = 0; i < block_node_ids.size(); ++i)
	    for(size_t j = block_node_ids[i][0]; j <= (size_t)block_node_ids[i][1]; ++j)
		block_pts_ids[i].insert(block_pts_ids[i].end(), node_pts_map[j].begin(), node_pts_map[j].end());

	return std::make_tuple(block_node_ids, block_pts_ids);
    }

    std::vector<int> getIntersectedFlag(const std::vector<std::vector<int> > &intersection_flag)
    {
	std::vector<int> intersected_flag(intersection_flag[0].size(), 0);
	for(size_t i = 0; i < intersection_flag[0].size(); ++i)
	    for(size_t j = 0; j < intersection_flag.size(); ++j)
		intersected_flag[i] += intersection_flag[j][i];

	return intersected_flag;
    }

    std::tuple<std::vector<int>, std::vector<double> >
    sortIntersectionFlag(const std::vector<int> &intersection_flag, const std::vector<double> &dist_along_ray)
    {
	std::vector<int> intersecting_ids;
	std::vector<double> dist_along_ray_intersections;
	for(size_t i = 0; i < intersection_flag.size(); ++i)
	    if (intersection_flag[i] == 1)
	    {
		intersecting_ids.push_back(i);
		dist_along_ray_intersections.push_back(dist_along_ray[i]);
	    }
    
	// get indices of ascending sort
	std::vector<int> sorted_ids(intersecting_ids.size());
	std::size_t n(0);
	std::generate(std::begin(sorted_ids), std::end(sorted_ids), [&]{ return n++; });

	std::sort( std::begin(sorted_ids), std::end(sorted_ids), 
		   [&](int i1, int i2) { return dist_along_ray_intersections[i1] < dist_along_ray_intersections[i2]; });

	std::vector<int> sorted_intersecting_ids(intersecting_ids.size());
	std::vector<double> sorted_dist_along_ray_intersections(dist_along_ray_intersections.size());
	for(size_t i = 0; i < sorted_ids.size(); ++i)
	{
	    sorted_intersecting_ids[i] = intersecting_ids[sorted_ids[i]];
	    sorted_dist_along_ray_intersections[i] = dist_along_ray_intersections[sorted_ids[i]];
	}
    
	return std::make_tuple(sorted_intersecting_ids, sorted_dist_along_ray_intersections);
    }

    std::tuple<int, bool>
    sampleHitId(const std::vector<double> &hit_prob_vec, const std::vector<int> &target_ids,
		const bool deterministic_sampling)
    {
	size_t n_targets = hit_prob_vec.size();
	std::random_device rd;
	std::mt19937 gen(rd());
   	std::uniform_real_distribution<> dis(0, 1);

	int hit_id;
	bool hit_bool;
	for(size_t i = 0; i < n_targets; ++i)
	{
	    if (deterministic_sampling)
	    {
		// std::cout << "determinstic hit id sampling!" << std::endl;
		return std::make_tuple(target_ids[0], true);
	    }

	    if (dis(gen) < hit_prob_vec[i])
	    {
		hit_id = target_ids[i];
		hit_bool = true;
		return std::make_tuple(hit_id, hit_bool);
	    }
	}
	hit_id = -1;
	hit_bool = false;
	return std::make_tuple(hit_id, hit_bool);
    }

    std::tuple<std::vector<double>, double> calcRayDirn(const std::vector<double> &ray_origin, const std::vector<double> &end_pt)
    {
	std::vector<double> ray_dirn(3,0);
	double meas_dist = 0;

	for(size_t i = 0; i < 3; ++i)
	{
	    ray_dirn[i] = end_pt[i] - ray_origin[i];
	    meas_dist += std::pow(ray_dirn[i], 2);
	}
	meas_dist = std::sqrt(meas_dist);
	for(size_t i = 0; i < 3; ++i)
	    ray_dirn[i] /= meas_dist;

	return std::make_tuple(ray_dirn, meas_dist);
    }
    
    std::vector<std::vector<double> >
    calcRayDirns(const std::vector<double> start_pt, const std::vector<std::vector<double> > end_pts)
    {
	size_t n_rays = end_pts.size();
	std::vector<std::vector<double> > ray_dirns(n_rays, std::vector<double>(3, 0));
	
	for(size_t i = 0; i < n_rays; ++i)
	{
	    for(size_t j = 0; j < 3; ++j)
		ray_dirns[i][j] = end_pts[i][j] - start_pt[j];

	    ray_dirns[i] = normalizeVec(ray_dirns[i]);
	}

	return ray_dirns;
    }

    void applyMaxRangeFilter(const std::vector<double> ray_origin,
			     std::vector<std::vector<double> > &sim_pts, std::vector<int> &hit_flag, 
			     const double max_range)
    {
	for(size_t i = 0; i < sim_pts.size(); ++i)
	    if (hit_flag[i])
	    {
		double range = euclideanDist(ray_origin, sim_pts[i]);
		if (range > max_range)
		    hit_flag[i] = 0;
	    }
    }
}

