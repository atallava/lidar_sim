#pragma once
#include <vector>
#include <string>

namespace lidar_sim {
    class OrientedBox {
    public:
	OrientedBox();
	void dispBox();
	void calcVertices();
	void padIntervals();
	bool checkPtInBox(const std::vector<double> &pt);

	std::vector<double> m_center;
	std::vector<std::vector<double> > m_axes;
	std::vector<std::vector<double> > m_intervals;
	std::vector<std::vector<double> > m_vertices;
	double m_interval_padding;
    };
}
