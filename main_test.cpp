#include <iostream>
#include "imu_calib.h"


int main(int argc, char* argv[])
{
	std::vector<double> acce = {-5.7461000e-02,  -6.7038000e-02,   9.8736880e+00};
	std::vector<double> gyro = { 5.2900900e-01,  -3.0653220e+00,  -8.0634000e-02};

	std::vector<double> acce_calib(3, 0), gyro_calib(3, 0);

	// calibrate accelerameter
	// parameter for HUAWEI MATE 7
	std::vector<std::vector<double> > Ta = {	{1,   0.00747551,   	0.00226059},
           				{0,        1, 		-0.000356226},
          				{-0,       0,            1}};

 	std::vector<std::vector<double> > Ka = {{0.982981,    0,       0},
      					{0, 		1.00974,     0},
      					{0,       	0,		 1.03968}};

    std::vector<double> acce_bias = {0.0836118,	-0.172405,	0.450842};

     // calibrate gyroscope
 	// parameter for HUAWEI MATE 7
    std::vector<std::vector<double> > Tg = {	{1.0,	  	0.0226504,	  0.0227521},
						{-0.0215821, 	1.0,	  0.00975126},
 						{0.0271248,  0.0365932,      1.0}};

    std::vector<std::vector<double> > Kg = {{1.00083,       0,       	0},
      					{0, 		1.03722,       	0},
      					{0,       0, 			1.01485}};

    std::vector<double> gyro_bias = {0.00154775,	-0.000749839,	0.00522445};

    Calibration calib;
    calib.set_acce_params(Ta, Ka, acce_bias);
    calib.set_gyro_params(Tg, Kg, gyro_bias);

	bool successed = calib.imu_calib(acce, gyro, acce_calib, gyro_calib);

	std::cout << acce_calib[0] << " " << acce_calib[1] << " " << acce_calib[2] << std::endl;
	std::cout << gyro_calib[0] << " " << gyro_calib[1] << " " << gyro_calib[2] << std::endl;
	
	return 0;
}