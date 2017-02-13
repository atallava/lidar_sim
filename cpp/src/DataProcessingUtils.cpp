#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <cstring>
#include <fstream>
#include <string>
#include <regex>

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>

namespace lidar_sim {
    std::string exec(const char* cmd) {
	char buffer[128];
	std::string result = "";
	std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
	if (!pipe) throw std::runtime_error("popen() failed!");
	while (!feof(pipe.get())) {
	    if (fgets(buffer, 128, pipe.get()) != NULL)
		result += buffer;
	}
	return result;
    }

    int getNumLinesInFile(std::string rel_path_file) {
	std::ostringstream ss;
	ss << "wc -l " << rel_path_file;
	std::string cmd = ss.str();
	std::string cmd_res = exec(cmd.c_str());
	char* buf = strdup(cmd_res.c_str());
	char* token = std::strtok(buf," ");
	int num_lines = atoi(token);

	return num_lines;
    }

    std::string genPCDHeader(int num_pts) {
	std::ostringstream header_ss;
	header_ss <<
	    "# .PCD v.7 - Point Cloud Data file format" << std::endl <<
	    "VERSION .7" << std::endl <<
	    "FIELDS x y z " << std::endl <<
	    "SIZE 4 4 4" << std::endl <<
	    "TYPE F F F" << std::endl <<
	    "COUNT 1 1 1" << std::endl <<
	    "WIDTH " << num_pts << std::endl <<
	    "HEIGHT 1" << std::endl <<
	    "VIEWPOINT 0 0 0 1 0 0 0" << std::endl <<
	    "POINTS " << num_pts << std::endl <<
	    "DATA ascii" << std::endl;
	return header_ss.str();
    }

    std::string getPtsLineFromSectionLine(std::string line)
    {
	std::istringstream iss(line);

	double data;
	// packet id
	iss >> data; 
	// packet timestamp
	iss >> data;
	iss >> data;

	double x, y, z;
	iss >> x;
	iss >> y;
	iss >> z;
	std::string line_pts = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
	
	return line_pts;
    }

    void subsampleFile(std::string rel_path_file, std::string rel_path_file_subsampled, int subsample_factor)
    {
	// open input file
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_file;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	// open output file
	std::ofstream subsampled_file(rel_path_file_subsampled);
	std::cout << "Writing to: " << rel_path_file_subsampled << std::endl;

	std::cout << "Subsampling factor: " << subsample_factor << std::endl;

	// subsample
	std::string current_line;
	int count = 0;
	while(std::getline(file,current_line))
	{
	    if (count % subsample_factor == 0)
		subsampled_file << current_line << std::endl;

	    count++;
	}

	// close files
	file.close();
	subsampled_file.close();
    }    

    void sectionOfSection(std::string rel_path_input, std::string rel_path_output, double start_time, double end_time)
    {
	// open input file
	std::ifstream input_file(rel_path_input);
	std::cout << "Reading from: " << rel_path_input << std::endl;
	if (!input_file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::ofstream output_file(rel_path_output);
	std::cout << "Writing to: " << rel_path_output << std::endl;

	std::string current_line;
	while(std::getline(input_file,current_line))
	{
	    double packet_timestamp_sec;
	    double packet_timestamp_nanosec;
	    double packet_timestamp;
	    int packet_id;
	    std::istringstream iss(current_line);

	    // packet id
	    iss >> packet_id; 

	    // packet timestamp
	    iss >> packet_timestamp_sec;
	    iss >> packet_timestamp_nanosec;
	    packet_timestamp = packet_timestamp_sec + packet_timestamp_nanosec*1e-9;

	    if ((packet_timestamp > start_time) && (packet_timestamp < end_time))
	    {
		output_file << current_line;
		output_file << std::endl;
	    }

	    if (packet_timestamp > end_time)
		break;
	}

	// close files
	input_file.close();
	output_file.close();
    }

    void prependPCDHeaderToFile(std::string rel_path_input, std::string rel_path_output)
    {
	std::ifstream input_file(rel_path_input);
	std::cout << "Reading from: " << rel_path_input << std::endl;
	if (!input_file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::ofstream output_file(rel_path_output);
	std::cout << "Writing to: " << rel_path_output << std::endl;

	int num_pts = getNumLinesInFile(rel_path_input); 
	std::string pcd_header = genPCDHeader(num_pts);
	
	// write header
	output_file << pcd_header;

	// write rest of input to output
	std::string current_line;
	while(std::getline(input_file,current_line))
	    output_file << current_line << std::endl;

	// close files
	input_file.close();
	output_file.close();
    }

    std::string genDetailLine(double packet_timestamp, std::vector<double> imu_pose, Eigen::Matrix<float,4,1> pt)
    {
	std::ostringstream ss;
	ss << packet_timestamp << " ";
	for (size_t i = 0; i < 6; ++i)
	    ss << imu_pose[i] << " ";
	for (size_t i = 0; i < 3; ++i)
	    ss << pt[i] << " ";
	ss << std::endl;

	return ss.str();
    }

    std::vector<std::vector<double> > loadPtsFromXYZFile(std::string rel_path_input)
    {
	std::ifstream input_file(rel_path_input);
	if (!input_file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::cout << "Reading pts from: " << rel_path_input << std::endl;

	std::vector<std::vector<double> > pts;
	
	std::string current_line;
	while(std::getline(input_file,current_line))
	{
	    std::istringstream iss(current_line);
	    double x, y, z;
	    iss >> x;
	    iss >> y;
	    iss >> z;
	    
	    std::vector<double> this_pt;
	    this_pt.push_back(x);
	    this_pt.push_back(y);
	    this_pt.push_back(z);

	    pts.push_back(this_pt);
  	}

	return pts;
    }

    std::tuple<std::vector<std::string>, std::vector<std::vector<double> > >
    loadAnnotations(const std::string rel_path_file)
    {
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_file;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::vector<std::string> object_classes;
	std::vector<std::vector<double> > object_posns;

	std::string current_line;
	while(std::getline(file,current_line))
	{
	    std::istringstream iss(current_line);

	    std::string object_class;
	    iss >> object_class;
	    object_classes.push_back(object_class);

	    std::vector<double> xy(2, 0);
	    iss >> xy[0]; iss >> xy[1];
	    object_posns.push_back(xy);
	}

	file.close();

	return std::make_tuple(object_classes, object_posns);
    }


    std::tuple<std::vector<double>, std::vector<double>, std::vector<double> >
    getVecsFromPts(const std::vector<std::vector<double> > &pts)
    {
	std::vector<double> x(pts.size(), 0);
	std::vector<double> y(pts.size(), 0);
	std::vector<double> z(pts.size(), 0);

	for(size_t i = 0; i < pts.size(); ++i)
	{
	    x[i] = pts[i][0];
	    y[i] = pts[i][1];
	    z[i] = pts[i][2];
	}

	return std::make_tuple(x, y, z);
    }

    std::vector<std::vector<double> > loadArray(std::string rel_path_file, int n_cols)
    {
	// open input file
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_file;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::vector<std::vector<double> > array;

	std::string current_line;
	while(std::getline(file,current_line))
	{
	    std::istringstream iss(current_line);
	    
	    std::vector<double> vec(n_cols, 0);
	    for(size_t i = 0; i < (size_t)n_cols; ++i)
		iss >> vec[i];

	    array.push_back(vec);
	}

	file.close();

	return array;
    }
    
    alglib::real_2d_array convertStlPtsToAlglibPts(const std::vector<std::vector<double> > &pts)
    {
	alglib::real_2d_array pts_alglib;
	pts_alglib.setlength(pts.size(), 3);

	for(size_t i = 0; i < pts.size(); ++i)
	    for(size_t j = 0; j < 3; ++j)
		pts_alglib[i][j] = pts[i][j];

	return pts_alglib;
    }

    std::vector<int> convertAlglib1DIntArrayToStlVector(const alglib::integer_1d_array &vec)
    {
	std::vector<int> vec_stl;
	
	for(size_t i = 0; i < (size_t)vec.length(); ++i)
	    vec_stl.push_back((int)vec[i]);

	return vec_stl;
    }

    Eigen::MatrixXd stlVecToEigen(const std::vector<double> &vec)
    {
	Eigen::MatrixXd vec_eigen(vec.size(), 1);
	for(size_t i = 0; i < vec.size(); ++i)
	    vec_eigen(i) = vec[i];

	return vec_eigen;
    }

    Eigen::MatrixXd stlArrayToEigen(const std::vector<std::vector<double> > &array)
    {
	Eigen::MatrixXd array_eigen(array.size(), array[0].size());
	for(size_t i = 0; i < array.size(); ++i)
	    for(size_t j = 0; j < array[0].size(); ++j)
		array_eigen(i,j) = array[i][j];

	return array_eigen;
    }

    std::vector<std::vector<double> > EigenToStlArray(const Eigen::MatrixXd &array)
    {
	std::vector<std::vector<double> > array_stl(array.rows(), 
						    std::vector<double>(array.cols()));
	for(size_t i = 0; i < (size_t)array.rows(); ++i)
	    for(size_t j = 0; j < (size_t)array.cols(); ++j)
		array_stl[i][j] = array(i,j);

	return array_stl;
    }

    std::vector<std::vector<int> > doubleToIntArray(const std::vector<std::vector<double> > &array)
    {
	std::vector<std::vector<int> > int_array(array.size(), std::vector<int>(array[0].size()));
	for(size_t i = 0; i < array.size(); ++i)
	    for(size_t j = 0; j < array[0].size(); ++j)
		int_array[i][j] = array[i][j];

	return int_array;
    }


    flann::Matrix<double> stlArrayToFlannMatrix(const std::vector<std::vector<double> > &array)
    {
	flann::Matrix<double> mat(new double[array.size()*array[0].size()], array.size(), array[0].size());
	for(size_t i = 0; i < array.size(); ++i)
	    for(size_t j = 0; j < array[0].size(); ++j)
		mat[i][j] = array[i][j];

	return mat;
    }

    std::vector<std::string> getPatternMatchingFiles(std::string rel_path_dir, boost::regex pattern)
    {
	std::vector< std::string > matching_filenames;

	boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
	for( boost::filesystem::directory_iterator i( rel_path_dir ); i != end_itr; ++i )
	{
	    if( !boost::filesystem::is_regular_file( i->status() ) ) continue;

	    std::string filename = i->path().filename().string();

	    if (boost::regex_search(filename, pattern))
		matching_filenames.push_back(filename);
	}

	return matching_filenames;
    }
}
