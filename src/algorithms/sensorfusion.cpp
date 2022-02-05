#include "sensorfusion.h"
#include <iostream>

SensorFusion::SensorFusion(){
	is_initialized_ = false;
	last_timestamp_ = 0.0;
	//initialization of measurement matix of lidar H_lidar
	H_lidar_ = Eigen::MatrixXd(2,5);
	H_lidar_ << 1, 0, 0, 0, 0,
		   0, 1, 0, 0, 0;

	//initialization of noise matix of lidar and radar R_lidar R_radar
	R_radar_ = Eigen::MatrixXd(3, 3);
	R_radar_ << 0.09, 0, 0,
		   0, 0.0009, 0,
		   0, 0, 0.09;
	R_lidar_ = Eigen::MatrixXd(2, 2);
	R_lidar_ << 0.0225, 0,
		    0, 0.0225;
	//std::cout << "Noise matrixs are initialized!" << std::endl;
}
SensorFusion::~SensorFusion()
{

}

void SensorFusion::Process(MeasurementPackage measurement_pack){
	//Initialization of Kalman Filter
	if (!is_initialized_){
		Eigen::VectorXd x_in = Eigen::VectorXd(5);
		// First data is lidar, position will be received without speed
		if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			float position_x = measurement_pack.raw_measurements_[0];
			float position_y = measurement_pack.raw_measurements_[1];
			x_in << position_x, position_y, 5, 0, 0;
			//std::cout << "lidar initialized!" << x_in <<std::endl;
		} else if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			//data transformation
			float rho = measurement_pack.raw_measurements_[0];
			float phi = measurement_pack.raw_measurements_[1];
			//float rho_dot = measurement_pack.raw_measurements_[2];
			float position_x = rho * cos(phi);
			float position_y = rho * sin(phi);
			//float velocity = rho_dot;
			x_in << position_x, position_y, 5, 0, 0;
			//std::cout << "radar initialized!" << std::endl;
		}
		//if (fabs(x_in(0))< 0.001) {
		//	x_in(0) = 0.001;
		//}
		//if (fabs(x_in(1))< 0.001) {
		//	x_in(1) = 0.001;
		//}
		
		// Initialization of kalman filter
		kf_.Initialization(x_in);
		
		// Set covariance matrix P
		Eigen::MatrixXd P_in = Eigen::MatrixXd(5, 5);
		P_in <<1.0, 0, 0, 0, 0,
		       0, 1.0, 0, 0, 0,
		       0, 0, 1.0, 0, 0,
		       0, 0, 0, 1.0, 0,
		       0, 0, 0, 0, 1.0;
		kf_.SetP(P_in);

		// Saving timestamp
		last_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;
		return;
	}
	long now_timestamp_ = measurement_pack.timestamp_;
	// 时间戳单位为微妙 转化为秒，更新delta_t	
	double delta_t = (now_timestamp_ - last_timestamp_) / 1000000.0;
	last_timestamp_ = now_timestamp_;
	//获取上一次的x值
	Eigen::VectorXd x_in = Eigen::VectorXd(5);
	x_in = kf_.GetX();

	// 设置状态转移矩阵
	// 判断偏航角是否为0
	Eigen::MatrixXd F_in = Eigen::MatrixXd(5,5);
	float v = x_in(2);
	float fai = x_in(3);
	float fai_dot = x_in(4);

	if (fabs(fai_dot) < 0.0001){
		F_in << 1.0, 0, cos(fai) * delta_t, -sin(fai) * v * delta_t, 0,
		        0, 1.0, sin(fai) * delta_t, cos(fai) * v * delta_t, 0,
			0, 0, 1.0, 0, 0,
			0, 0, 0, 1.0, delta_t,
			0, 0, 0, 0, 1.0;
			//std::cout << "F initialized(zero)" << F_in << std::endl;
	}
       	else if(fabs(fai_dot) >= 0.0001) {
		float F13 = 1/fai_dot * (sin(fai + fai_dot*delta_t) - sin(fai));
		float F23 = 1/fai_dot * (-cos(fai + fai_dot*delta_t) + cos(fai));
		float F14 = -v * F23;
		float F24 = v * F13;
		float F15 = -v/fai_dot*F13 + v*delta_t/fai_dot*cos(fai + fai_dot*delta_t);
		float F25 = -v/fai_dot*F23 + v*delta_t/fai_dot*sin(fai + fai_dot*delta_t);
		F_in << 1.0, 0, F13, F14, F15,
		        0, 1.0, F23, F24, F25,
			0, 0, 1.0, 0, 0,
			0, 0, 0, 1.0, delta_t,
			0, 0, 0, 0, 1.0;
			//std::cout << "F initialized(not zero)" << F_in << std::endl;
	}
	if (x_in(1) < -2){
		//std::cout << "Current fai" << fai << std::endl;
		//std::cout << "Current F" << F_in << std::endl;
	}
	kf_.SetF(F_in);
	// 设置过程噪音矩阵
	Eigen::MatrixXd Q_in = Eigen::MatrixXd(5,5);
	Eigen::MatrixXd G = Eigen::MatrixXd(5,2);
	Eigen::MatrixXd Q_v = Eigen::MatrixXd(2,2);
	Q_v << 4, 0,
	       0, 0.09;
	G << 0.5 * delta_t * delta_t * cos(fai), 0,
	     0.5 * delta_t * delta_t * sin(fai), 0,
	     delta_t, 0,
	     0, 0.5 * delta_t * delta_t,
	     0, delta_t;
	Q_in = G * Q_v * G.transpose();
	kf_.SetQ(Q_in);

	kf_.Prediction(delta_t);
	if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		kf_.SetH(H_lidar_);
		kf_.SetR(R_lidar_);
		kf_.KFUpdate(measurement_pack.raw_measurements_);
		
	} else if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		kf_.SetR(R_radar_);
		kf_.EKFUpdate(measurement_pack.raw_measurements_);
	}
}

