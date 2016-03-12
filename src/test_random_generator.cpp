#include "random_vector_generator.h"
#include <iostream>


int main(int argc, char** argv)
{
    Eigen::Vector3d mean;
    Eigen::Vector3d var;
    var[0] = var[1] = var[2] = 1.0;
    RandomVectorGenerator<3> vector_generator(mean, var);
    
    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d v = vector_generator.generate_vector();
        std::cout << "[" << v[0] << "; " << v[1] << "; " << v[2] << "]" << std::endl;
    }

    std::cout << std::endl 
              << "##########################################################" 
              << std::endl << std::endl;

    Eigen::Vector3d axis_mean;
    axis_mean[2] = 1.0;
    Eigen::Vector3d axis_var;
    axis_var[0] = axis_var[1] = 0.1;
    double angle_mean = 0.0;
    double angle_var  = 1.0;
    RandomAngleAxisGenerator angle_axis_generator(angle_mean, angle_var, axis_mean, axis_var);

    for (int i = 0; i < 100; i++)
    {
        Eigen::AngleAxisd a = angle_axis_generator.generate_angle_axis();
	std::cout << "[" << a.axis()[0] << "; " << a.axis()[1] << "; " << a.axis()[2] << "], " 
	          << a.angle() << std::endl;
    }

    return 0;
}
