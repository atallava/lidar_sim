#include <string>
#include <lidar_sim/Test.h>

using namespace lidar_sim;

int main() {
    Test t = Test();
    // std::string rel_path_pose_log = "../data/taylorJune2014/Pose/dummy_pose_log.txt";
    // t.testGetPoseAtTime(rel_path_pose_log, 40.01);
    // t.testEigenmvn();
    t.testFLANN();
}
