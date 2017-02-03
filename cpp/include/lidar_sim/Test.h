#pragma once
#include <string>

namespace lidar_sim {
    class Test {
    public:
        bool testVizPCD(std::string file_name, int is_rgb);
        bool testVizPoints();
	bool testPoseServer(std::string rel_path_pose_log);
	bool testGetPoseAtTime(std::string rel_path_pose_log, double t);
	bool testCovMat();
	bool testEigenmvn();
	bool testFLANN();
	bool testLibgp();
	bool testAlglibRbf();
	bool testCgal();
	bool testCgalIntersection();
	bool testEigenvalues();
	bool testPtsLine();
	bool testVizEllipsoid();
    };
}
