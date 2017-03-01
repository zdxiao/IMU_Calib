#include "imu_calib.h"

void M3xV3(double M3[3][3], double V3[3], double ret[3])
{
	for(int i = 0; i < 3; ++i)
	{
		ret[i] = M3[i][0] * V3[0] + M3[i][1] * V3[1] + M3[i][2] * V3[2];
	}
}

void calib_trans(double raw_data[3], double T[3][3], double K[3][3], double bias[3], double calib_data[3])
{
	double unbias_data[3] = {0};
	for(int i = 0; i < 3; ++i)
		unbias_data[i] = raw_data[i] - bias[i];
	double tmp[3];
	M3xV3(K, unbias_data, tmp);
	M3xV3(T, tmp, calib_data);
}

void imu_calib(double acce[3], double gyro[3], double acce_calib[3], double gyro_calib[3])
{
	// calibrate accelerameter
	// parameter for HUAWEI MATE 7
	double Ta[3][3] = {	1,   0.00747551,   	0.00226059,
           				0,        1, 		-0.000356226,
          				-0,       0,            1};

 	double Ka[3][3] = {0.982981,    0,       0,
      					0, 		1.00974,     0,
      					0,       	0,		 1.03968};

    double acce_bias[3] = {0.0836118,	-0.172405,	0.450842};

    calib_trans(acce, Ta, Ka, acce_bias, acce_calib);

    // calibrate gyroscope
 	// parameter for HUAWEI MATE 7
    double Tg[3][3] = {	1.0,	  	0.0226504,	  0.0227521,
						-0.0215821, 	1.0,	  0.00975126,
 						0.0271248,  0.0365932,      1.0};

    double Kg[3][3] = {1.00083,       0,       	0,
      					0, 		1.03722,       	0,
      					0,       0, 			1.01485};

    double gyro_bias[3] = {0.00154775,	-0.000749839,	0.00522445};

    calib_trans(gyro, Tg, Kg, gyro_bias, gyro_calib);
}