#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <cstring>
#include <fstream>
#include <string>

#include <lidar_sim/DataProcessingUtils.h>

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

    void subsampleFile(std::string rel_path_file, std::string rel_path_file_subsampled, int subsample_factor)
    {
	// open input file
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;

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
}
