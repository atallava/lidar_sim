#include <iostream>
#include <cmath>
#include <ctime>

int main()
{
    clock_t start_time = clock();

    const int size = 256;
    double sin_table[size];

// #pragma omp parallel for
    for(size_t i = 0; i < size; ++i)
    {
	sin_table[i] = std::sin(2*M_PI*i/size);
    }

    std::cout << sizeof(sin_table) << std::endl;
    double elapsed_time = (clock() - start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s. " << std::endl;

    return(1);
}
