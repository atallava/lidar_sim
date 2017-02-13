#include <iostream>
#include <math.h>
#include <cmath>
#include <chrono>
#include <random>
#include <time.h>
#include <algorithm>

#include <Eigen/Dense>

#include <lidar_sim/OrientedBox.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

OrientedBox::OrientedBox()
{
    m_interval_padding = 0.2;
}
    
void OrientedBox::dispBox()
{
    std::cout << "m_center: " << std::endl;
    dispVec(m_center);
    std::cout << "m_axes: " << std::endl;
    dispMat(m_axes);
    std::cout << "m_intervals: " << std::endl;
    dispMat(m_intervals);
}

void OrientedBox::calcVertices()
{
    for(size_t i = 0; i < 2; ++i)
    {
	std::vector<double> v0(2, 0);
	for(size_t k = 0; k < 2; ++k)
	    v0[k] += m_intervals[0][i]*m_axes[0][k];
	    
	for(size_t j = 0; j < 2; ++j) 
	{
	    std::vector<double> v1(2, 0);
	    for(size_t k = 0; k < 2; ++k)
		v1[k] += m_intervals[1][j]*m_axes[1][k];

	    std::vector<double> vertex(2, 0);
	    for(size_t k = 0; k < 2; ++k)
		vertex[k] = m_center[k] + v0[k] + v1[k];

	    m_vertices.push_back(vertex);
	}
    }

    // flip last two rows to maintain cyclic order
    std::vector<double> v = m_vertices[2];
    m_vertices[2] = m_vertices[3];
    m_vertices[3] = v;
}

void OrientedBox::padIntervals()
{
    for(size_t j = 0; j < 2; ++j)
    {
	m_intervals[j][0] -= m_interval_padding;
	m_intervals[j][1] += m_interval_padding;
    }
}

bool OrientedBox::checkPtInBox(const std::vector<double> &pt)
{
    std::vector<double> centered_pt(2, 0);
    for(size_t i = 0; i < 2; ++i)
	centered_pt[i] = pt[i] - m_center[i];

    std::vector<double> projns(2, 0);
    for(size_t i = 0; i < 2; ++i)
	projns[i] = dotProduct(centered_pt, m_axes[i]);

    bool condn1 = (m_intervals[0][0] <= projns[0]);
    bool condn2 = (projns[0] <= m_intervals[0][1]);
    bool condn3 = (m_intervals[1][0] <= projns[1]);
    bool condn4 = (projns[1] <= m_intervals[1][1]);

    bool res = condn1 && condn2 && condn3 && condn4;
    return res;
}

