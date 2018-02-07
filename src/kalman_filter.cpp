#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  //init Kalman Filter data members
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}



//predict function is same for RADAR and LASER
void KalmanFilter::Predict() {
  /**
    * predict the state
  */
	//predict state using state transition matrix and earlier state
	x_ = F_ * x_;
	//Transpose of state transition matrix
	MatrixXd Ft = F_.transpose();
	//covariance matrix , Q is process noise mtx
	P_ = F_ * P_ * Ft + Q_;
}


//update function for LASER, linear kalman filter
void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */

  //calc predicted state, H is measurement function which is linear for linear kalman function
  VectorXd z_pred = H_ * x_;
  //y is difference between predicted and measured state
  VectorXd y = z - z_pred;

  //aply kalman filter equations and update state and covariance mtx with kalman gain,K
  // the statements below are common to linear and extended kalman, can be created in a function
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


//update function for RADAR, extended kalman for non linear eq
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */

  //convert cartesian coords of current state to polar coords to compare against Radar measurement
  float rho_sq=pow(x_(0),2)+pow(x_(1),2);
  float rho=sqrt(rho_sq);
  //float phi=atan(x_(1)/x_(0));
  float phi=atan2(x_(1),x_(0));
  //float rhodot=(x_(0)*x_(2)+x_(1)*x_(3))/rho;
  float rhodot;
  //if rho is too small (<0.0001, 10000th of radian), set rhodot to 0
  if (fabs(rho) < 0.0001) {
	  rhodot=0.0;
  	  } else {
  		//else rhodot is px*vx+py*vy/rho;
  		rhodot=(x_(0)*x_(2)+x_(1)*x_(3))/rho;
  	  }
  //z_pred is 3x1 mtx as opposed to 2x1 in linear kalman
  VectorXd z_pred(3);
  z_pred << rho,phi,rhodot;

  //difference of predicted vs measured state values
  VectorXd y = z - z_pred;
  // phi should be between -pi and +pi
    while(y(1) < -M_PI) {
    	//cout << "phi LT PI: "<<y(1) << endl;
      	y(1) = y(1) + 2 * M_PI;
      	//cout << "phi normalized to PI: "<<y(1) << endl;
    	}


    while(y(1) > M_PI) {
    	//cout << "phi GT PI: " << y(1) << endl;
      	y(1) = y(1) - 2 * M_PI;
      	//cout << "phi normalized to PI: "<<y(1) << endl;
    	}

  //apply kalman equations, Jacobian measureemnt function will be calculated in FusionEKF ProcessMeasurement
  //method aand assigned to H_ member of ekf_ object
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
