#include <iostream>
#include "imu_calib.h"

int main(int argc, char* argv[])
{
	double acce[3] = {-5.7461000e-02,  -6.7038000e-02,   9.8736880e+00};
	double gyro[3] = { 5.2900900e-01,  -3.0653220e+00,  -8.0634000e-02};

	double acce_calib[3], gyro_calib[3];

	imu_calib(acce, gyro, acce_calib, gyro_calib);

	std::cout << acce_calib[0] << " " << acce_calib[1] << " " << acce_calib[2] << std::endl;
	std::cout << gyro_calib[0] << " " << gyro_calib[1] << " " << gyro_calib[2] << std::endl;
	
	return 0;
}