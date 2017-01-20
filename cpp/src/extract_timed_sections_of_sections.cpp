#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <stdexcept>

#include <Eigen/Dense>

#include <lidar_sim/Utils.h>
#include <lidar_sim/FrameTimeServer.h>

using namespace lidar_sim;

int main() {
    std::string rel_path_section_pre = "../data/taylorJune2014/sections/world_frame/section_";
    std::string rel_path_section_post = "_world_frame_subsampled.xyz";
    
    std::string rel_path_section_timed_pre = "../data/taylorJune2014/sections/world_frame/section_";
    std::string rel_path_section_timed_post = "_world_frame_subsampled_timed.xyz";

    std::string rel_path_frame_time_log = "../data/taylorJune2014/lowres/AVTCamera_156580/Camera_156580_Timestamp.txt";
    FrameTimeServer frame_time_server(rel_path_frame_time_log);

    int subsection_start_frame_id = 16170;
    int subsection_end_frame_id = 16290;
    double subsection_start_time = frame_time_server.getTimeAtFrameId(subsection_start_frame_id);
    double subsection_end_time = frame_time_server.getTimeAtFrameId(subsection_end_frame_id);
    
    // loop over sections
    clock_t start_time = clock();

    // section file
    int section_id = 8;
    
    std::ostringstream ss;
    ss << rel_path_section_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_section_post;
    std::string rel_path_section = ss.str();

    // section timed file
    ss.str("");
    ss.clear();
    ss << rel_path_section_timed_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_section_timed_post;
    std::string rel_path_section_timed = ss.str();

    // write
    sectionOfSection(rel_path_section, rel_path_section_timed, subsection_start_time, subsection_end_time);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
