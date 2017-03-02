/** @brief act the calibration to the imu raw data by the parameter from imu-tk
/*  	The calibration model is imu_calib = T * K * (imu_raw -  bias)
*	@author: Shon Xiao, at Tencent Inlab
*	@date: March 2nd, 2017
*/

#ifndef IMU_CALIB
#define IMU_CALIB

#include <vector>

class Calibration
{
public:
	// constructor
	Calibration(){};
	// destructor
	~Calibration(){};
	// calibrate accelerate
	bool imu_calib_acce(const std::vector<double> &acce, std::vector<double> &acce_calib);
	// calibrate gyroscope
	bool imu_calib_gyro(const std::vector<double> &gyro, std::vector<double> &gyro_calib);
	// calibrate both
	bool imu_calib(const std::vector<double> &acce, const std::vector<double> &gyro, 
		std::vector<double> &acce_calib, std::vector<double> &gyro_calib);
	// set accelerate calibration parameters
	void set_acce_params(const std::vector<std::vector<double> > &T, const std::vector<std::vector<double> > &K, const std::vector<double> &bias);
	// set gyroscope calibration parameters
	void set_gyro_params(const std::vector<std::vector<double> > &T, const std::vector<std::vector<double> > &K, const std::vector<double> &bias);
	// get accelerate calibration parameters
	void get_acce_params(std::vector<std::vector<double> > &T, std::vector<std::vector<double> > &K, std::vector<double> &bias);
	// get gyroscope calibration parameters
	void get_gyro_params(std::vector<std::vector<double> > &T, std::vector<std::vector<double> > &K, std::vector<double> &bias);

private:
	// accelerate parameters
	std::vector<std::vector<double> > Ta, Ka; 
	std::vector<double> acce_bias;
	// gyroscope parameters
	std::vector<std::vector<double> > Tg, Kg; 
	std::vector<double> gyro_bias;
	
	// do the T * K * (x - bias) transform
void calib_trans(const std::vector<double> &raw_data, const std::vector<std::vector<double> > &T, const std::vector<std::vector<double> > &K, 
	const std::vector<double> &bias, std::vector<double> &calib_data);
	// simple function for matrix 3by3 to mutiply std::vector 3
void M3xV3(const std::vector<std::vector<double> > &M3, const std::vector<double> &V3, std::vector<double> &ret);
};

#endif