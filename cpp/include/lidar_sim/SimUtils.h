#pragma once

#include <lidar_sim/SectionModelSim.h>

namespace lidar_sim {
    // should be working with references here. ignoring because sim is created once in script (for now)
    SectionModelSim createSectionModelSimObject(int section_models_id, std::string sim_version, bool deterministic_sim);
}

