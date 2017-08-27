#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <lidar_sim/OptimProgress.h>
#include <lidar_sim/DataProcessingUtils.h>

using namespace lidar_sim;

OptimProgress::OptimProgress() :
    m_verbose(false)
{
}

void OptimProgress::save(const std::string rel_path_output)
{
    std::ofstream file(rel_path_output);
    if (m_verbose)
	std::cout << "Writing optim progress to: " << rel_path_output << std::endl;

    for (size_t i = 0; i < m_x.size(); ++i)
	file << getStrFromVec(m_x[i]) << " "
	     << m_J[i] << " " << m_t[i] << std::endl;

    file.close();
}
