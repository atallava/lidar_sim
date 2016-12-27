#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <cstring>

#include <lidar_sim/Utils.h>

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
}
