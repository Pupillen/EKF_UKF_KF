/*** 
 * @Author: ztr
 * @Date: 2022-02-25 23:35:11
 * @LastEditTime: 2022-02-26 00:15:33
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /EKF_KF_UKF/include/ekf.hpp
 */
#ifndef EKF_H__
#define EKF_H__
#include <eigen3/Eigen/Core>
#include <iostream>
using namespace std;
using namespace Eigen;
class EKF
{
    public:
        EKF();
        ~EKF();
    void InitSystem(int n_states, int n_outputs, const MatrixXd& Q, const MatrixXd& R);
    virtual VectorXd f(const VectorXd &x, const VectorXd &u);
    virtual VectorXd h(const VectorXd &x);
    void InitSystemState(const VectorXd& x0);
    void InitSystemStateCovariance(const MatrixXd& P0);
    void EKalmanf(const VectorXd& u);
    void EKalmanf(const VectorXd& z, const VectorXd& u);
    const VectorXd& GetCurrentState() const;
    const VectorXd& GetCurrentOutput() const;
    const VectorXd& GetCurrentEstimatedState() const;
    const VectorXd& GetCurrentEstimatedOutput() const;

    private:
void CalcF(const VectorXd &x, const VectorXd &u);
void CalcH(const VectorXd &x);
    MatrixXd F_;// Jacobian of F 
    MatrixXd H_;// jacobian of H
    MatrixXd Q_;// Process noise covariance
    MatrixXd R_;// Measurement noise covariance
    MatrixXd sqrt_Q_;//Process noise stdev
    MatrixXd sqrt_R_;//Measurement noise stdev
    MatrixXd P_p_;//State covariance after a priori update
    MatrixXd P_m_;//State covariance after measurement update
    VectorXd v_;// Gaussian process noise
    VectorXd w_;// Gaussian measurement noise
    VectorXd x_m_;//State vector after measurement update
    VectorXd x_p_;//State vector after a priori update
    VectorXd z_m_;//Estimated output
    double epsilon_; // Very small number for Jacobian
protected:
    int nStates_;   // Number of the states
    int nOutputs_;  // Number of outputs   
};
#endif
