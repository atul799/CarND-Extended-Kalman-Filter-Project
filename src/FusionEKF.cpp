#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement noise matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement noise matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //initialize H_laser_ --> 2x4
  H_laser_ << 1,0,0,0,
              0,1,0,0;

  //initilaize Hj --> 3x4
  Hj_<<0,0,0,0,
       0,0,0,0,
       0,0,0,0;

  //init covarence mat
  //cov of vx-vx and vy-vy initialized to 1000, can be set at lower/rand values ??
  ekf_.P_ = MatrixXd(4, 4);
  //ekf_.P_ << 1, 0, 0, 0,
  //             0, 1, 0, 0,
  //             0, 0, 1000, 0,
  //             0, 0, 0, 1000;
  //ekf_.P_ << 1, 0, 0, 0,
  //            0, 1, 0, 0,
  //            0, 0, 1, 0,
  //            0, 0, 0, 1;

  //init state trans matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix. --> cov mtx,P is initialized in contructor
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    

    //init state vector, starting with 1 for px,py,vx,vy, is reinitialized with measurement data anyways
    //in following section
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    //if sensor data is from RADAR, measurement data --> rho/phi and rhodot
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho=measurement_pack.raw_measurements_[0];
      float phi=measurement_pack.raw_measurements_[1];
      float rhodot=measurement_pack.raw_measurements_[2];

      //convert polar to cartesian cords
      //rho,phi is  specified anticlockwise from x (considered vertical) and y considered
      //horiz pointing to left
      float px= rho*cos(phi);
      float py=rho*sin(phi);
      //float vx=0;
      //float vy=0;
      float vx=rhodot*cos(phi);
      float vy=rhodot*sin(phi);
	  cout <<"Radar init, px,py,vx,vy"<<ekf_.x_<<endl;
      ekf_.x_<<px,py,vx,vy;
      //cout <<"Radar init, px,py,vx,vy"<<ekf_.x_ <<endl;
    }
    //if sensor data is from LIDAR, measurement data --> px and py directly
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      

      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      //cout <<"Lidar init, px,py,vx,vy"<<ekf_.x_ <<endl;
    }



    //save initial time stamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    cout <<"Init Done"<<endl;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //calculate dt
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  //cout << "DT is: "<<dt<<endl;


  //state transition matrix re-init to add dt for vx.dt/vy.dt for position update
  ekf_.F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
  //cout<<"F mat:  "<<ekf_.F_<<endl;

  float noise_ax=9.0;
  float noise_ay=9.0;
  //process noise
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << (pow(dt,4)/4)*noise_ax, 0, (pow(dt,3)/2)*noise_ax, 0,
        0, (pow(dt,4)/4)*noise_ay, 0, (pow(dt,3)/2)*noise_ay,
        (pow(dt,3)/2)*noise_ax, 0, pow(dt,2)*noise_ax, 0,
        0, (pow(dt,3)/2)*noise_ay, 0, pow(dt,2)*noise_ay;
  //cout<<"Q mat:  "<<ekf_.Q_<<endl;
  //cout <<"Predict Start: "<< ekf_.x_<<endl;
  //if value of dt less than a 1e-6 of a uS, forego Predict step
  //if (dt > 0.000001) {
  //	  //cout << "DT is > 1uS: "<<dt<<" : " << measurement_pack.sensor_type_<<endl;
	  ekf_.Predict();
  //	  } else {
  //		  //cout << "DT is < 1uS: "<<dt<< " : " << measurement_pack.sensor_type_<<endl;
  //	  }
  cout <<"Predict done: "<<endl;
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

	// Radar updates
    //Jacobian for H
    ekf_.H_=tools.CalculateJacobian(ekf_.x_);
    //cout << "Jacobian Res" << ekf_.H_<<endl;
    ekf_.R_=R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates

	ekf_.H_=H_laser_;
    ekf_.R_=R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }
  cout <<"Update Done"<<endl;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}