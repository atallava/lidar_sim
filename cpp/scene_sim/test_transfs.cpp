#include <string>
#include <vector>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

int main(int argc, char**argv)
{
    std::vector<std::vector<double> > pts_1{{0, 0, 0},
	{1, 1, 0}};

    std::vector<double> xyz{0, 0, 0};
    double theta = 3.1415;
    Eigen::Matrix4d T_1_to_2 = transfz(xyz, theta);
    std::vector<std::vector<double> > pts_2 = applyTransfToPts(pts_1, T_1_to_2);
    dispMat(pts_1);
    dispMat(pts_2);
}
