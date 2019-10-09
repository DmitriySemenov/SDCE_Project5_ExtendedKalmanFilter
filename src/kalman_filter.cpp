#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();

	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

	// Calculate z_pred using nonlinear h(x)
	VectorXd z_pred = h_radar(x_);
	VectorXd y = z - z_pred;
	
	if (y(1) > M_PI)
		y(1) -= 2 * M_PI;
	else if (y(1) < -M_PI)
		y(1) += 2 * M_PI;

	// Uses Jacobian Matrix here. FusionEKF updates H
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
}

VectorXd KalmanFilter::h_radar(const VectorXd& x) {
	// Map cartesian positions and velocities to polar range, angle, and range rate
	VectorXd z_pred = VectorXd(3);

	float px = x(0);
	float py = x(1);
	float vx = x(2);
	float vy = x(3);
	float px_2 = px * px;
	float py_2 = py * py;

	float ro = sqrt(px_2 + py_2);
	float theta = atan2(py, px);
	float ro_dot = 0;
	if (ro > 0.001) {
		ro_dot = (px * vx + py * vy) / ro;
	}
	
	z_pred << ro,
						theta,
						ro_dot;

	return z_pred;
}
