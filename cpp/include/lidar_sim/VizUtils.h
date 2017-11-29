#pragma once
#include <string>
#include <iostream>

#include <Eigen/Dense>

namespace lidar_sim {
    template<typename T>
    void dispVec(std::vector<T> vec)
    {
	for(size_t i = 0; i < vec.size(); ++i)
	    std::cout << vec[i] << " ";
	std::cout << std::endl;
    }

    template<typename T>
    void dispMat(std::vector<std::vector<T> > mat)
    {
	for(size_t i = 0; i < mat.size(); ++i)
	{
	    for(size_t j = 0; j < mat[i].size(); ++j)
		std::cout << mat[i][j] << " ";
	    std::cout << std::endl;
	}
    }

    void dispHorizontalLine(int length = 50);
}
