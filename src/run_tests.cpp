#include <string>
#include <lidar_sim/Test.h>

using namespace lidar_sim;

int main() {
    Test t = Test();
    t.testVizPCD("data/dataset_2.pcd");
    //t.testVizPoints();
}
