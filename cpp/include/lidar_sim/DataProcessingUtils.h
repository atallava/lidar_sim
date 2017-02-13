#pragma once
#include <string>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

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
    std::vector<std::vector<double> > loadPtsFromXYZFile(std::string rel_path_file);
    std::tuple<std::vector<std::string>, std::vector<std::vector<double> > >
	loadAnnotations(const std::string rel_path_annotations);

    template<typename T>
	void writePtsToXYZFile(const std::vector<std::vector<T> > &pts, const std::string rel_path_output, const int pt_dim = 3)
    {
	std::ofstream file(rel_path_output);
	std::cout << "Writing pts to: " << rel_path_output << std::endl;

	for(size_t i = 0; i < pts.size(); ++i)
	{
	    std::ostringstream ss;
	    for(size_t j = 0; j < pt_dim; ++j)
		ss << pts[i][j] << " ";
	    ss << std::endl;
	    
	    file << ss.str();
	}

	file.close();
    }

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
	std::vector<std::vector<T> > subsampleArray(const std::vector<std::vector<T> > &array,
	    int subsample_factor = 10)
    {
	std::vector<std::vector<T> > array_subsampled;
	for(size_t i = 0; i < array.size(); ++i)
	    if ((i % subsample_factor) == 0)
		array_subsampled.push_back(array[i]);

	return array_subsampled;
    }

    std::vector<std::string> getPatternMatchingFiles(std::string rel_path_dir, boost::regex pattern);
}
