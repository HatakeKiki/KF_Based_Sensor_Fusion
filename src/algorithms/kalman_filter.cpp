#include "kalman_filter.h"
#include <iostream>

// Constructor(构造函数Default constructor with no input parameters)
KalmanFilter::KalmanFilter(){
	is_initialized_ = false;
}

KalmanFilter::~KalmanFilter()
{

}

Eigen::VectorXd KalmanFilter::GetX(){
	return x_;
}

bool KalmanFilter::IsInitialized(){
	return is_initialized_;
}


void KalmanFilter::Initialization(Eigen::VectorXd x_in){
	x_ = x_in;
	is_initialized_ = true;
}

void KalmanFilter::SetF(Eigen::MatrixXd F_in){
	F_ = F_in;
}

void KalmanFilter::SetP(Eigen::MatrixXd P_in){
	P_ = P_in;
}

void KalmanFilter::SetQ(Eigen::MatrixXd Q_in){
	Q_ = Q_in;
}

void KalmanFilter::SetH(Eigen::MatrixXd H_in){
	H_ = H_in;
}

void KalmanFilter::SetR(Eigen::MatrixXd R_in){
	R_ = R_in;
}

void KalmanFilter::Prediction(double delta_t){
	//x_ = F_* x_;
	Eigen::VectorXd x_add = Eigen::VectorXd(5);
	if (fabs(x_(4)) >= 0.0001){
		x_add(0) = x_(2)/x_(4)*(sin(x_(3)+x_(4)*delta_t) - sin(x_(3)));
		x_add(1) = x_(2)/x_(4)*(-cos(x_(3)+x_(4)*delta_t) + cos(x_(3)));
	       	x_add(2) = 0;
		x_add(3) = x_(4) * delta_t;
		x_add(4) = 0;
	} else if (fabs(x_(4)) < 0.0001){
		x_add(0) = x_(2) * cos(x_(3)) * delta_t;
		x_add(1) = x_(2) * sin(x_(3)) * delta_t;
		x_add(2) = 0;
		x_add(3) = x_(4)*delta_t;
		x_add(4) = 0;
	}
	x_ = x_ + x_add;
	x_(3) = AngleRange(x_(3));
	Eigen::MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::KFUpdate(Eigen::VectorXd z){
	Eigen::VectorXd y = z - H_ * x_;
	Eigen::MatrixXd Ht = H_.transpose();
	Eigen::MatrixXd S = H_ * P_ * Ht + R_;
	Eigen::MatrixXd Si = S.inverse();
	Eigen::MatrixXd K =  P_ * Ht * Si;
	//std::cout << z(0) << "	"
	//          << z(1) << std::endl;
	x_ = x_ + (K * y);
	x_(3) = AngleRange(x_(3));
	int x_size = x_.size();
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::EKFUpdate(Eigen::VectorXd z){
	double px = x_(0);
	double py = x_(1);
	double v = x_(2);
	double fai = x_(3);
	double rho = sqrt(px * px + py * py);
	double theta = atan2(py, px);
	double vx = v * cos(fai);
	double vy = v * sin(fai);
	double rho_dot = 0;
	if (rho < 0.0001){
		rho_dot = 0;
	} else if (rho >= 0.0001){
		rho_dot = (px * vx + py * vy) / rho;
	}
	Eigen::VectorXd h = Eigen::VectorXd(3);
	theta = AngleRange(theta);
	h << rho, theta, rho_dot;
	z(1) = AngleRange(z(1));
        Eigen::VectorXd y = z -h;
	y(1) = AngleRange(y(1));
	JacobMatrixCalculation();
	//std::cout << "Error y:" << y << std::endl;
	//std::cout << "Measurement z:" << z << std::endl;
	//std::cout << "Current h:" << h  << std::endl;

	Eigen::MatrixXd Ht = H_.transpose();
	Eigen::MatrixXd S = H_ * P_ * Ht + R_;
	Eigen::MatrixXd Si = S.inverse();
	Eigen::MatrixXd K =  P_ * Ht * Si;
	x_ = x_ + (K * y);
	x_(3) = AngleRange(x_(3));
	int x_size = x_.size();
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::JacobMatrixCalculation(){
	Eigen::MatrixXd Hj = Eigen::MatrixXd(3,5);
	float px = x_(0);
	float py = x_(1);
	float v = x_(2);
	float yaw = x_(3);
	yaw = AngleRange(yaw);
	double vx = v * cos(yaw);
	double vy = v * sin(yaw);
	float rho = sqrt(px * px + py * py);
	Hj << px / rho, py / rho, 0.0, 0.0, 0.0,
	      -py / (rho * rho), px / (rho * rho), 0.0, 0.0, 0.0,
	      py * (vx * py - vy * px) / (rho * rho * rho), px * (-vx * py + vy * px) / (rho * rho * rho), (px * cos(yaw) + py * sin(yaw)) / rho, (-px * sin(yaw) + py * cos(yaw)) * v / rho, 0.0;
	if (fabs(rho) < 0.0001) {
		Hj << 0.0, 0.0, 0.0, 0.0, 0.0,
		      0.0, 0.0, 0.0, 0.0, 0.0,
		      0.0, 0.0, 0.0, 0.0, 0.0;
	}
	H_ = Hj;
	//std::cout << "Jacobian Matix of Radar" << Hj << std::endl;
	return;
}

float KalmanFilter::AngleRange(float angle){
	pi = 3.1415926;
	while(angle > pi){
		angle = angle - 2 * pi;
	}
	while (angle < -pi){
		angle = angle + 2 * pi;
	}
	return angle;
}

