// test some phrases of code that appear in the sim 

#include <iostream>
#include <vector>
#include <omp.h>

int main() 
{

    clock_t start_time = clock();


    std::vector<int> vec;
    size_t n_vec = 10000;
    vec.reserve(n_vec);

    std::cout << "after reserve, vec[20]: " << vec[20] << std::endl;

#pragma omp parallel for
    for(size_t i = 0; i < n_vec; ++i)
	vec[i] = i;

    std::cout << "after assign, vec[20]: " << vec[20] << std::endl;

    double elapsed_time = (clock() - start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s. " << std::endl;

    return(1);
}
