#pragma once
#include <string>

#include <Eigen/Dense>

#include "stdafx.h"
#include "dataanalysis.h"

namespace lidar_sim {
    std::string exec(const char* cmd);	
    int getNumLinesInFile(std::string rel_path_file);
    std::string genPCDHeader(int num_pts);
    void subsampleFile(std::string rel_path_file, std::string rel_path_file_subsampled, int subsample_factor);
    void prependPCDHeaderToFile(std::string rel_path_input, std::string rel_path_output);
    std::string genDetailLine(double packet_timestamp, std::vector<double> imu_pose, Eigen::Matrix<float,4,1> pt);
    void sectionOfSection(std::string rel_path_input, std::string rel_path_output, double start_time, double end_time);
    std::vector<std::vector<double> > loadPtsFromXYZFile(std::string rel_path_file);
    alglib::real_2d_array convertStlPtsToAlglibPts(std::vector<std::vector<double> > pts);
    void dispPt(std::vector<double> pt);
}
