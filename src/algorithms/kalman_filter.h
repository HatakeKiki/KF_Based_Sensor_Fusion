#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter{
private:
	// flag of initialization
	bool is_initialized_;
	float pi;
	float angle;
	// state vector of kalman filter
	Eigen::VectorXd x_;
	// state transmission matrix
	Eigen::MatrixXd F_;
	// state covariance matrix
	Eigen::MatrixXd P_;
	// process covariance matrix
	Eigen::MatrixXd Q_;
	// measurement matrix
	Eigen::MatrixXd H_;
	// measurement covariance matrix
	Eigen::MatrixXd R_;
	void JacobMatrixCalculation();

public:
	KalmanFilter();
	~KalmanFilter();
	Eigen::VectorXd GetX();
	bool IsInitialized();
	void Initialization(Eigen::VectorXd x_in);
	void SetF(Eigen::MatrixXd F_in);
	void SetP(Eigen::MatrixXd P_in);
	void SetQ(Eigen::MatrixXd Q_in);
	void SetH(Eigen::MatrixXd H_in);
	void SetR(Eigen::MatrixXd R_in);
	void Prediction(double delta_t);
	void KFUpdate(Eigen::VectorXd z);
	void EKFUpdate(Eigen::VectorXd z);
	float AngleRange(float angle);
};

#endif
