/**
/**
 * @file main4.cpp
 * @author Auralius Manurung
 * @date 18 Apr 2015
 * 
 * @brief Example for the extended Kalman filter.
 * 
 * @section DESCRIPTION
 * This example is taken from <a href="http://ch.mathworks.com/matlabcentral/fileexchange/38302-kalman-filter-package/content//Kalman%20Filter%20Package/Examples/ExtendedKalmanFilterDemo.m">here</a>.
 */

#include <fstream>

#include "ekf.hpp"

/// @cond DEV
/*
 * Class EKF needs to be derived, two virtual functions are provided in 
 * which system model and output model are described.
 */
using namespace my_filter;
class MyEKF: public EKF
{
public:  
  virtual VectorXd f(const VectorXd& x, const VectorXd& u) {
    VectorXd xk(nOutputs_);
    xk(0) = sin(x(1) * u(0));
    xk(1) = x(1);
    return xk;
  }
  
  virtual VectorXd h(const VectorXd& x) {
    VectorXd zk(nOutputs_);
    zk(0) = x(0);
    zk(1) = x(1);
    return zk;
  }
};
/// @endcond

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  /* 
   * Log the result into a tab delimitted file, later we can open 
   * it with Matlab. Use: plot_data4.m to plot the results.
   */
/*   ofstream log_file;
#ifdef _WIN32
  log_file.open("..\\bin\\log_file4.txt");
#else
  log_file.open("log_file4.txt");
#endif */
  
  int n_states = 2;
  int n_outputs = 2;
  MatrixXd Q(2, 2);
  MatrixXd R(2, 2);
  Q<<0.001,0,
         0,0;
  R<<0.1,0,
     0,0.01;
  VectorXd x0(2);
  x0<< 0,1 * M_PI / 500;
  VectorXd u(1);

   
  MyEKF myekf;
  myekf.InitSystem(n_states, n_outputs, Q, R);
  myekf.InitSystemState(x0);
  
  for (int k = 0; k < 1000; k ++) {
    u(0) = k;
    myekf.EKalmanf(u);
    VectorXd x(2),x_m(2),z(1);
    x = myekf.GetCurrentState();
    x_m = myekf.GetCurrentEstimatedState();
    z = myekf.GetCurrentOutput();
    //'Position Measurements', 'True Position', 'Position Estimates'
  }
  return 0;
}
