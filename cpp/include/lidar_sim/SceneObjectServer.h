#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    class SceneObjectServer {
    public:
	SceneObjectServer();
	void loadPrimitiveEllipsoidModels(const std::string rel_path_primitive_models_dir);
	boost::regex genRegexPatternForClassName(const std::string class_name);
	void getPrimitiveElementIds(const std::string rel_path_primitive_models_dir);
	int getElementIdFromFilename(const std::string filename, const std::string class_name);
	std::string genRelPathPrimitiveEllipsoidModels(const std::string rel_path_primitive_models_dir,
						       const std::string class_name, const int element_id);

	EllipsoidModels m_ellipsoid_models;
	std::vector<std::string> m_primitive_class_names;
	std::vector<std::vector<int> > m_primitive_element_ids;
	std::vector<std::vector<EllipsoidModels> > m_primitive_ellipsoid_models;

    private:
	int m_debug_flag;
	std::string m_model_files_ext;
    };
}
