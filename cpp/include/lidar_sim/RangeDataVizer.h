#pragma once
#include <vector>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <Eigen/Dense>

#include <lidar_sim/EllipsoidVtkActorServer.h>
#include <lidar_sim/PointsVtkActorServer.h>
#include <lidar_sim/LineVtkActorServer.h>
#include <lidar_sim/EllipsoidModelUtils.h>

namespace lidar_sim {
    class RangeDataVizer {
    public:
	RangeDataVizer();
	void vizEllipsoidModels(EllipsoidModels ellipsoid_models);
	void vizEllipsoidModels(EllipsoidModels ellipsoid_models, Pts pts);
	void vizPts(std::vector<std::vector<double> > pts);
	void takeItAway(std::vector<vtkSmartPointer<vtkActor> > actors);
	void vizComparePts(std::vector<std::vector<double> > pts1,
			   std::vector<std::vector<double> > pts2);

	PointsVtkActorServer m_points_actor_server;
	LineVtkActorServer m_line_actor_server;
	EllipsoidVtkActorServer m_ellipsoid_actor_server;

    private:
	
    };
}

