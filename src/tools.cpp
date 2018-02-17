#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
    * RMSE is used to find diff between estimated state values and ground truth
  */
	VectorXd rmse(4);
    rmse << 0,0,0,0;

	// check the validity of the following inputs:
        //  * the estimation vector size should not be zero
        //  * the estimation vector size should equal ground truth vector size
        // ... your code here

    if (estimations.size() ==0 || ground_truth.size()== 0 || estimations.size() != ground_truth.size() ) {
                cout <<"one of the estimation or ground truth is either 0 or their size not equal";
                return rmse;
        }


    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
    	// estimations[i] and ground_truth[i] is  vector of size 4 containg,px,py,vx,vy
    	//difference of each state element is taken then squared and added to cumulative sum of rmse
        VectorXd diff=estimations[i]-ground_truth[i];
        diff=diff.array()*diff.array();
        rmse +=diff;

    }

    //calculate the mean
    rmse=rmse/estimations.size();

    //calculate the squared root

	rmse=rmse.array().sqrt();
    //return the result
	//record rmse
	//cout << "RMSE: "<<rmse<<endl;

    return rmse;

}

// this function calculates Jacobian for non-linear kalman measurement function of Radar data to linearize H_ matrix
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	Hj<< 0,0,0,0,
	     0,0,0,0,
	     0,0,0,0;
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    //cout<<px<<", "<<py<<", "<<vx<<", "<<vy<<endl;

    //square sum for eucladean distance
    float polar_dist_sq=pow(px,2)+pow(py,2);
    //sqrt to get eucladean (polar) distance
    float polar_dist=sqrt(polar_dist_sq);
    //3/2 power --> polar_dist_sq*polar_dist
    //float polar_dist_3=pow(polar_dist,3);
    float polar_dist_3=polar_dist_sq*polar_dist;
    //cout << polar_dist_sq<<", "<<polar_dist<<" ,"<<polar_dist_3<<endl;


    //check division by zero/extremely small number for ploar_dist
    //if (polar_dist_sq < 0.0001  || polar_dist< 0.0001 || polar_dist_3 < 0.0001 ) {
    if (fabs(polar_dist) < 0.0001)  {
        cout << "Attempting divide by zero, returning without jacobian calc"<<endl;

        return Hj;

    }
    //compute the Jacobian matrix

    Hj << px/polar_dist,py/polar_dist,0,0,
          -1*py/polar_dist_sq,px/polar_dist_sq,0,0,
          py*(vx*py-vy*px)/polar_dist_3,px*(vy*px-vx*py)/polar_dist_3,px/polar_dist,py/polar_dist;


    return Hj;

}
