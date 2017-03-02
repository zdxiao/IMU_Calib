/** @brief act the calibration to the imu raw data by the parameter from imu-tk
/*  	The calibration model is imu_calib = T * K * (imu_raw -  bias)
*	@author: Shon Xiao, at Tencent Inlab
*	@date: March 2nd, 2017
*/
#include "imu_calib.h"

void Calibration::M3xV3(const std::vector<std::vector<double> > &M3, const std::vector<double> &V3, std::vector<double> &ret)
{
	for(int i = 0; i < 3; ++i)
	{
		ret[i] = M3[i][0] * V3[0] + M3[i][1] * V3[1] + M3[i][2] * V3[2];
	}
}

void Calibration::calib_trans(const std::vector<double> &raw_data, const std::vector<std::vector<double> > &T, const std::vector<std::vector<double> > &K, 
	const std::vector<double> &bias, std::vector<double> &calib_data)
{
	std::vector<double> unbias_data(3, 0);
	for(int i = 0; i < 3; ++i)
		unbias_data[i] = raw_data[i] - bias[i];
	std::vector<double> tmp(3, 0);
	M3xV3(K, unbias_data, tmp);
	M3xV3(T, tmp, calib_data);
}

bool Calibration::imu_calib(const std::vector<double> &acce, const std::vector<double> &gyro, std::vector<double> &acce_calib, std::vector<double> &gyro_calib)
{
	if(Ta.size() == 3 && Ka.size() == 3 && acce_bias.size() == 3)
	{
		calib_trans(acce, Ta, Ka, acce_bias, acce_calib);
	}
    else
    {
    	return false;
    }

    if(Tg.size() == 3 && Kg.size() == 3 && gyro_bias.size() == 3)
	{
		calib_trans(gyro, Tg, Kg, gyro_bias, gyro_calib);
	}
	else
	{
		return false;
	}
	return true;
}

// calibrate accelerate
bool Calibration::imu_calib_acce(const std::vector<double> &acce, std::vector<double> &acce_calib)
{
	if(Ta.size() == 3 && Ka.size() == 3 && acce_bias.size() == 3)
	{
		calib_trans(acce, Ta, Ka, acce_bias, acce_calib);
	}
    else
    {
    	return false;
    }
    return true;
}
// calibrate gyroscope
bool Calibration::imu_calib_gyro(const std::vector<double> &gyro, std::vector<double> &gyro_calib)
{
    if(Tg.size() == 3 && Kg.size() == 3 && gyro_bias.size() == 3)
	{
		calib_trans(gyro, Tg, Kg, gyro_bias, gyro_calib);
	}
	else
	{
		return false;
	}
	return true;
}
// set accelerate calibration parameters
void Calibration::set_acce_params(const std::vector<std::vector<double> > &T, const std::vector<std::vector<double> > &K, 
	const std::vector<double> &bias)
{
	Ta = T;
	Ka = K;
	acce_bias = bias;
}
// set gyroscope calibration parameters
void Calibration::set_gyro_params(const std::vector<std::vector<double> > &T, const std::vector<std::vector<double> > &K, const std::vector<double> &bias)
{
	Tg = T;
	Kg = K;
	gyro_bias = bias;
}
// get accelerate calibration parameters
void Calibration::get_acce_params(std::vector<std::vector<double> > &T, std::vector<std::vector<double> > &K, std::vector<double> &bias)
{
	T = Ta;
	K = Ka;
	bias = acce_bias;
}
// get gyroscope calibration parameters
void Calibration::get_gyro_params(std::vector<std::vector<double> > &T, std::vector<std::vector<double> > &K, std::vector<double> &bias)
{
	T = Tg;
	K = Kg;
	bias = gyro_bias;
}
