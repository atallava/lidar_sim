#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

LaserIntrinsics::LaserIntrinsics() 
{
    alpha_vec = {30.67,-9.33,-29.33,-8.00,-28.00,-6.66,-26.66,-5.33,-25.33,-4.00,-24.00,-2.67,-22.67, -1.33,-21.33,0.00,-20.00,1.33,-18.67,2.67,-17.33,4.00,-16.00,5.33,-14.67,6.67,-13.33,8.00,-12.00,9.33,-10.67,10.67};
    for(size_t i = 0; i < alpha_vec.size(); ++i)
	alpha_vec[i] = deg2rad(alpha_vec[i]);
    
    double theta_resn = 20;
    double angle = 1;
    while (angle < 360)
    {
	theta_vec.push_back(deg2rad(angle));
	angle += theta_resn;
    }
    
    min_range = 0;
    max_range = 70;
}

LaserExtrinsics::LaserExtrinsics()
{
    Eigen::MatrixXd T(4,4);
    T << 
    	0.0086996955871186, 0.9999621097991535, -0.0003070223393724, -0.0090499101707984,
    	-0.9999567242503523, 0.0087006599957041, 0.0032936517945554, -0.1016216668883396,
    	0.0032961982944134, 0.0002783552847679, 0.9999945287826025, 0.5000000000000000,
    	0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
    T_laser_imu = T;
}

LaserCalibParams::LaserCalibParams() :
    intrinsics(),
    extrinsics()
{
}













