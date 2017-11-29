#include <iostream>
#include <sstream>

#include <lidar_sim/VizUtils.h>

namespace lidar_sim {
    void dispHorizontalLine(int length)
    {
    	std::cout << std::endl;
    	for(size_t i = 0; i < (size_t)length; ++i)
    	    std::cout << "-";
    	std::cout << std::endl << std::endl;
    }
}
