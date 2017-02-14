#include <algorithm>
#include <chrono>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/regex.hpp>

#include <lidar_sim/SceneObjectServer.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>

using namespace lidar_sim;

SceneObjectServer::SceneObjectServer() :
    m_debug_flag(0),
    m_model_files_ext(".txt")
{    
    m_primitive_class_names = std::vector<std::string> {"low_shrub", "low_shrub_patch", "medium_shrub", 
						    "medium_shrub_patch", "thin_shrub", "large_shrub", 
						    "large_shrub_patch", "medium_tree", "large_tree"};
    m_primitive_element_ids = std::vector<std::vector<int> > (m_primitive_class_names.size(), std::vector<int>());
    m_primitive_ellipsoid_models.resize(m_primitive_class_names.size());
}

void SceneObjectServer::loadPrimitiveEllipsoidModels(const std::string rel_path_primitive_models_dir)
{
    // todo: state the assumption on file structure
    getPrimitiveElementIds(rel_path_primitive_models_dir);
    for(size_t i = 0; i < m_primitive_class_names.size(); ++i)
    {
	m_primitive_ellipsoid_models[i].resize(m_primitive_element_ids[i].size());
	for(size_t j = 0; j < m_primitive_element_ids[i].size(); ++j)
	{
	    std::string rel_path_models = genRelPathPrimitiveEllipsoidModels(rel_path_primitive_models_dir,
									     m_primitive_class_names[i], m_primitive_element_ids[i][j]);
	    m_primitive_ellipsoid_models[i][j] =  loadEllipsoidModelsFromFile(rel_path_models);
	}
    }
}

boost::regex SceneObjectServer::genRegexPatternForClassName(const std::string class_name)
{
    std::ostringstream ss;
    ss << class_name << "_[0-9]+";
    boost::regex pattern(ss.str());
    
    return pattern;
}

void SceneObjectServer::getPrimitiveElementIds(const std::string rel_path_primitive_models_dir)
{
    for(size_t i = 0; i < m_primitive_class_names.size(); ++i)
    {	
	boost::regex pattern = genRegexPatternForClassName(m_primitive_class_names[i]);
	std::vector<std::string> matching_filenames = 
	    getPatternMatchingFiles(rel_path_primitive_models_dir, pattern);
	std::vector<int> ids;
	for(size_t j = 0; j < matching_filenames.size(); ++j)
	    ids.push_back(
		getElementIdFromFilename(matching_filenames[j], m_primitive_class_names[i]));
	std::sort(ids.begin(), ids.end());
	m_primitive_element_ids[i] = ids;
    }
}

int SceneObjectServer::getElementIdFromFilename(const std::string filename, const std::string class_name)
{
    size_t posn1 = filename.find(class_name);
    size_t posn2 = filename.find(m_model_files_ext);
    size_t n_dig = (posn2 - posn1 - class_name.size() - 1); // accounting for _ after class name in filename
    return std::stoi(filename.substr(posn1 + class_name.size() + 1, n_dig));
}

std::string SceneObjectServer::genRelPathPrimitiveEllipsoidModels(const std::string rel_path_primitive_models_dir,
								  const std::string class_name, const int element_id)
{
    std::ostringstream ss;
    ss << rel_path_primitive_models_dir << "/" << class_name << "_" << element_id << m_model_files_ext;

    return ss.str();
}

EllipsoidModels SceneObjectServer::genObjectModelsAtPosn(const std::string class_name, const std::vector<double> &posn)
 {
     // pick element at random
     int class_id = classIdFromName(class_name);
     std::vector<int> shuffled_ids(m_primitive_ellipsoid_models[class_id].size());
     std::iota(shuffled_ids.begin(), shuffled_ids.end(), 0);
     uint64_t seed = std::chrono::duration_cast<std::chrono::nanoseconds>
	 (std::chrono::steady_clock::now().time_since_epoch()).count();
     std::shuffle(std::begin(shuffled_ids), std::end(shuffled_ids),
		  std::default_random_engine(seed));
     int primitive_id = shuffled_ids[0];

     // shift models
     EllipsoidModels models = m_primitive_ellipsoid_models[class_id][primitive_id];
     setEllipsoidModelsCentroid(models, posn);

     return models;
 }

void SceneObjectServer::setEllipsoidModelsCentroid(EllipsoidModels &models, 
						   const std::vector<double> &posn)
{
    // gather means
    std::vector<std::vector<double> > mus;
    for(size_t i = 0; i < models.size(); ++i)
	mus.push_back(models[i].mu);
    std::vector<double> current_centroid = calcPtsMean(mus);

    for(size_t i = 0; i < models.size(); ++i)
	for(size_t j = 0; j < 3; ++j)
	    models[i].mu[j] = posn[j] + (models[i].mu[j] - current_centroid[j]);
}

int SceneObjectServer::classIdFromName(const std::string class_name)
{
    int class_id = -1;
    for(size_t i = 0; i < m_primitive_class_names.size(); ++i)
	if (m_primitive_class_names[i].compare(class_name) != 0)
	    class_id = i;
    
    return class_id;
}
