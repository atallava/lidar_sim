#pragma once
#include <string>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>

#include <Eigen/Dense>

#include <boost/regex.hpp>

#include "stdafx.h"
#include "dataanalysis.h"

#include <flann/flann.hpp>

namespace lidar_sim {
    std::string exec(const char* cmd);	

    // file processing
    int getNumLinesInFile(std::string rel_path_file);
    std::string genPCDHeader(int num_pts);
    std::string getPtsLineFromSectionLine(std::string line);
    void subsampleFile(std::string rel_path_file, std::string rel_path_file_subsampled, int subsample_factor);
    void prependPCDHeaderToFile(std::string rel_path_input, std::string rel_path_output);
    std::string genDetailLine(double packet_timestamp, std::vector<double> imu_pose, Eigen::Matrix<float,4,1> pt);
    void sectionOfSection(std::string rel_path_input, std::string rel_path_output, double start_time, double end_time);
    std::vector<std::vector<double> > loadPtsFromXYZFile(std::string rel_path_file, int verbose = 1);
    std::tuple<std::vector<std::string>, std::vector<std::vector<double> > >
	loadAnnotations(const std::string rel_path_annotations);

    template<typename T>
	void writePtsToXYZFile(const std::vector<std::vector<T> > &pts, const std::string rel_path_output, int verbose = 1)
    {
	std::ofstream file(rel_path_output);
	if (verbose)
	    std::cout << "Writing pts to: " << rel_path_output << std::endl;

	for(size_t i = 0; i < pts.size(); ++i)
	{
	    std::ostringstream ss;
	    for(size_t j = 0; j < pts[i].size(); ++j)
		ss << pts[i][j] << " ";
	    ss << std::endl;
	    
	    file << ss.str();
	}

	file.close();
    }

    template<typename T>
	void writeVecToFile(const std::vector<T> &vec, const std::string rel_path_output)
    {
	std::ofstream file(rel_path_output);
	std::cout << "Writing vec to: " << rel_path_output << std::endl;

	std::ostringstream ss;
	for(size_t i = 0; i < vec.size(); ++i)
	    ss << vec[i] << " ";
	ss << std::endl;
	    
	file << ss.str();
	file.close();
    }

    void writeStringToFile(const std::string str, const std::string rel_path_output);

    void writeQueriedBlocks(const std::string rel_path_file, const std::vector<int> &triangle_block_ids, 
			    const std::vector<int> &ellipsoid_block_ids);
    std::tuple<std::vector<int>, std::vector<int> >
	readQueriedBlocks(const std::string rel_path_input);


    std::tuple<std::vector<double>, std::vector<double>, std::vector<double> >
	getVecsFromPts(const std::vector<std::vector<double> > &pts);

    std::vector<std::vector<double> > loadArray(std::string rel_path_file, int n_cols);

    // data types conversions
    alglib::real_2d_array convertStlPtsToAlglibPts(const std::vector<std::vector<double> > &pts);
    std::vector<int> convertAlglib1DIntArrayToStlVector(const alglib::integer_1d_array &vec);

    Eigen::MatrixXd stlVecToEigen(const std::vector<double> &vec);
    Eigen::MatrixXd stlArrayToEigen(const std::vector<std::vector<double> > &array);
    std::vector<std::vector<double> > EigenToStlArray(const Eigen::MatrixXd &array);
    std::vector<std::vector<int> > doubleToIntArray(const std::vector<std::vector<double> > &array);

    flann::Matrix<double> stlArrayToFlannMatrix(const std::vector<std::vector<double> > &array);    

    template<typename T>
    	std::vector<std::vector<T> > flannMatrixToStlArray(const flann::Matrix<T> &mat_flann)
    {
    	std::vector<std::vector<T> > array(mat_flann.rows, std::vector<T>(mat_flann.cols));
    	for(size_t i = 0; i < mat_flann.rows; ++i)
    	    for(size_t j = 0; j < mat_flann.cols; ++j)
    		array[i][j] = mat_flann[i][j];

    	return array;
    }

    template<typename T>
	std::string getStrFromVec(const std::vector<T> &vec)
    {
	std::ostringstream ss;
	for (size_t i = 0; i < (vec.size()-1); ++i)
	    ss << vec[i] << " ";
	ss << vec.back();

	return ss.str();
    }

    template<typename T>
	std::vector<std::vector<T> > subsampleArray(const std::vector<std::vector<T> > &array,
	    int subsample_factor = 10)
    {
	std::vector<std::vector<T> > array_subsampled;
	for(size_t i = 0; i < array.size(); ++i)
	    if ((i % subsample_factor) == 0)
		array_subsampled.push_back(array[i]);

	return array_subsampled;
    }

    // useful when sometimes passing a single element to a function that needs a 
    // vector of elements as input
    template<typename T>
	std::vector<T> wrapDataInVec(T data)
    {
	std::vector<T> vec;
	vec.push_back(data);
	
	return vec;
    }

    template<typename T>
	std::vector<T> getUniqueSortedVec(std::vector<T> vec)
    {
	std::sort(vec.begin(), vec.end());
	vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
	return vec;
    }

    // mimicking matlab mat(:)
    template<typename T>
	std::vector<T> convertArrayToVec(std::vector<std::vector<T> > array)
    {
	std::vector<T> vec;
	for(size_t i = 0; i < array.size(); ++i)
	    vec.insert(vec.end(), array[i].begin(), array[i].end());

	return vec;
    }

    std::vector<double> convertIntVecToDoubleVec(std::vector<int> vec);

    std::vector<std::string> getPatternMatchingFiles(std::string rel_path_dir, boost::regex pattern);
    // should both these be handled by one case?
    std::tuple<std::vector<std::string>, std::vector<std::vector<std::string> > >
	getPatternMatchingFiles(std::string rel_path_dir, boost::regex pattern, int num_captures);

    std::vector<int> getGroundBlockIds(const std::string rel_path_ground_models_dir, const int section_id);
    std::vector<int> getNonGroundBlockIds(const std::string rel_path_non_ground_models_dir, const int section_id);
    std::vector<int> getEllipsoidModelBlockIds(const std::string rel_path_ellipsoid_models_dir, const int section_id);
    std::vector<int> getTriangleModelBlockIds(const std::string rel_path_triangle_models_dir, const int section_id);
    
    std::vector<int> getObjectMeshIds(const std::string rel_path_object_meshes_dir);
    
    std::string getDateString(const std::string format = "%d%m%y");

    // the path helpers
    std::string genPathSection(const int section_id);
    std::string genPathPosesLog();
    std::string genPathImuPosnNodes(const int section_id);
    std::string genPathBlockNodeIdsGround(const int section_id);
    std::string genPathBlockNodeIdsNonGround(const int section_id);
    std::string genPathNonGroundBlockPts(const int section_id, const int block_id);
    std::string genPathGroundBlockPts(const int section_id, const int block_id);
    std::string genPathHgModelsDir(const int section_id, const std::string sim_version);
    std::string genPathRealPtsRef(const int section_id, const std::string sim_type, 
				  const std::string sim_version, const std::string query_type, const int tag);
    std::string genPathSimPts(const int section_id, const std::string sim_type, 
			      const std::string sim_version, const std::string query_type, const int tag);
    std::string genPathSimDetail(const int section_id, const std::string sim_type, 
				 const std::string sim_version, const std::string query_type, const int tag);
}
