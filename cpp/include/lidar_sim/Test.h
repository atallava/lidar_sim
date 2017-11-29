#pragma once
#include <string>

namespace lidar_sim {
    class Test {
    public:
	bool testPoseServer(std::string rel_path_pose_log);
	bool testGetPoseAtTime(std::string rel_path_pose_log, double t);
	bool testCovMat();
	bool testEigenmvn();
	bool testFLANN();
	bool testAlglibRbf();
	bool testCgal();
	bool testCgalIntersection();
	bool testEigenvalues();
	bool testPtsLine();
	bool testGetEllipseTransform();
	bool testFlannDatasetWrapper();
	bool testEllipsoidSimNbrServer();
    };
}
