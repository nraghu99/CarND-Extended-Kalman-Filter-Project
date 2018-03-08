#include <iostream>
#include "tools.h"

#define EPS 0.0001 // A very small number
#define EPS2 0.0000001

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // TODO: YOUR CODE HERE
    
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    // ... your code here
    if ((estimations.size() != ground_truth.size()) || (estimations.size() ==0)) {
        cout << "Invalid data " <<  endl;
        return rmse;
    }
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        VectorXd residual  = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    
    //calculate the mean
    rmse =  rmse / estimations.size();
    
    //calculate the squared root
    rmse = rmse.array().sqrt();
    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    
    MatrixXd Hj(3,4);
    
    if ( x_state.size() != 4 ) {
        cout << "ERROR - CalculateJacobian () - The state vector must have size 4." << endl;
        return Hj;
    }
    
    
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    
    //check division by zero
    // Deal with the special case problems
    if (fabs(px) < EPS and fabs(py) < EPS){
        px = EPS;
        py = EPS;
    }
  
    //compute the Jacobian matrix
    float c1 = px * px + py * py;
    
    // Check division by zero
    if(fabs(c1) < EPS2){
        c1 = EPS2;
    }
        
    float c2 = sqrt(c1);
    float c3 = c1 * c2;
    
        
        
    Hj << px/c2, py / c2, 0.0,0.0,
    -py / c1, px/c1,0.0,0.0,
    py * (vx*py - vy*px) / c3, px * (vy*px-vx*py) / c3,px/c2, py / c2;
   
    return Hj;
}
