#include <string>
#include <lidar_sim/Test.h>

using namespace lidar_sim;

int main() {
    Test t = Test();
    std::string rel_path_pcd = std::string("../data/taylorJune2014/sections/world_frame/section_") + \
	"03" + "_world_frame_subsampled.pcd";
    int is_rgb = 0;
    t.testVizPCD(rel_path_pcd, is_rgb);
}
