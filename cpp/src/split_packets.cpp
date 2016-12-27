#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <algorithm>
#include <iomanip>

int main() {
    std::string rel_path_velo_data = "../data/taylorJune2014/Velodyne/Velodyne_Rip.xyz";
    std::ifstream infile(rel_path_velo_data);
    
    // TODO: read these values from file
    std::vector<std::vector<int>> section_packet_ids
    {	
    	{508777,      628023,      966997,     1101873,     1314446,     1535404,     1663680,     1889454,      2137531,     2421155,     2973268,     3670531,     3845148,     3995101},
    	{626214,      770745,     1086258,     1312638,     1512565,     1661871,     1824466,     1990654, 2419384,     2619274,     3426109,     3786182,     3993292,     4095674}};
    
    int num_sections = section_packet_ids[0].size();
    bool writing_out = false;
    int section_count = 1;
    std::string rel_path_section_pre = "../data/taylorJune2014/Velodyne/sections/section_";
    std::string rel_path_section_post = ".xyz";
    std::string rel_path_section;
    std::ofstream section_file;

    std::string current_line;
    int packet_id;

    clock_t startTime = clock();
    while (std::getline(infile,current_line)) 
    {
    	// get packet number
    	std::istringstream iss(current_line);
    	iss >> packet_id;

    	// section start
    	if ((std::find(section_packet_ids[0].begin(), section_packet_ids[0].end(), packet_id) != section_packet_ids[0].end()) \
    	    && (!writing_out))
    	{
    	    // open section file
    	    std::ostringstream ss;
    	    ss << rel_path_section_pre << std::setw(2) << std::setfill('0') << section_count << rel_path_section_post;
    	    rel_path_section = ss.str();
    	    section_file.open(rel_path_section);

    	    // turn writing on
    	    writing_out = true;
	    
    	    std::cout << "Starting write: " << rel_path_section << std::endl;
    	}
    	// section end
    	else if ((std::find(section_packet_ids[1].begin(), section_packet_ids[1].end(), packet_id) != section_packet_ids[1].end()) \
    	    && (writing_out))
    	{
    	    std::cout << "Ending write: " << rel_path_section << std::endl;

    	    // close buffer
    	    section_file.close();
	    
    	    // turn writing off
    	    writing_out = false;
	    
    	    // update section count
    	    section_count = section_count+1;
    	    if (section_count > num_sections)
    		break;
    	}
    	else
    	{
    	    // do nothing
    	}

    	// write to section file
    	if (writing_out)
    	    section_file << current_line << std::endl;
    }

    double elapsedTime = (clock()-startTime)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsedTime << "s." << std::endl;

    return 0;
}
