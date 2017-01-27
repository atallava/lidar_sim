#pragma once
#include <vector>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <Eigen/Dense>

#include <lidar_sim/EllipsoidVtkActorServer.h>
#include <lidar_sim/PointsVtkActorServer.h>
#include <lidar_sim/EllipsoidsModelUtils.h>

namespace lidar_sim {
    class RangeDataVizer {
    public:
	RangeDataVizer();
	void vizEllipsoidModels(EllipsoidModels ellipsoid_models);
	void vizEllipsoidModels(EllipsoidModels ellipsoid_models, Pts pts);
	void takeItAway(std::vector<vtkSmartPointer<vtkActor> > actors);

    private:
	PointsVtkActorServer m_points_actor_server;
	EllipsoidVtkActorServer m_ellipsoid_actor_server;
    };
}

