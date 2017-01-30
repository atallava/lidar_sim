#pragma once
#include <string>

#include <Eigen/Dense>

#include "stdafx.h"
#include "dataanalysis.h"

#include <flann/flann.hpp>

namespace lidar_sim {
    std::string exec(const char* cmd);	

    int getNumLinesInFile(std::string rel_path_file);
    std::string genPCDHeader(int num_pts);
    void subsampleFile(std::string rel_path_file, std::string rel_path_file_subsampled, int subsample_factor);
    void prependPCDHeaderToFile(std::string rel_path_input, std::string rel_path_output);
    std::string genDetailLine(double packet_timestamp, std::vector<double> imu_pose, Eigen::Matrix<float,4,1> pt);
    void sectionOfSection(std::string rel_path_input, std::string rel_path_output, double start_time, double end_time);
    std::vector<std::vector<double> > loadPtsFromXYZFile(std::string rel_path_file);
    void writePtsToXYZFile(std::vector<std::vector<double> > pts, std::string rel_path_output);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double> >
	getVecsFromPts(const std::vector<std::vector<double> > &pts);

    alglib::real_2d_array convertStlPtsToAlglibPts(std::vector<std::vector<double> > pts);

    Eigen::MatrixXd stlVecToEigen(std::vector<double> vec);
    Eigen::MatrixXd stlArrayToEigen(std::vector<std::vector<double> > array);
    std::vector<std::vector<double> > EigenToStlArray(Eigen::MatrixXd array);

    flann::Matrix<double> stlArrayToFlannMatrix(std::vector<std::vector<double> > array);    
    std::vector<std::vector<double> > flannMatrixToStlArray(flann::Matrix<double> mat);

    template<typename T>
	std::vector<std::vector<T> > subsampleArray(std::vector<std::vector<T> > array,
	    int subsample_factor = 10)
    {
	std::vector<std::vector<T> > array_subsampled;
	for(size_t i = 0; i < array.size(); ++i)
	    if ((i % subsample_factor) == 0)
		array_subsampled.push_back(array[i]);

	return array_subsampled;
    }
}
