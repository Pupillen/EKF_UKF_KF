/*** 
 * @Author: ztr
 * @Date: 2022-02-25 23:35:11
 * @LastEditTime: 2022-02-26 14:18:34
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /EKF_KF_UKF/include/ekf.hpp
 */
#ifndef EKF_H__
#define EKF_H__
#include <eigen3/Eigen/Core>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;
namespace my_filter
{
   /*  x_k = f(x_{k-1}, u_{k-1}) + v_{k-1}   Gaussian process v~N(0,Q)
    z_k = h(x_k) + w_k                    Gaussian process w~N(0,R)
    偏f偏x=F   Jacobian of F
    偏h偏x=H   Jacobian of H */
  class EKF
{
    public:
        EKF();
        ~EKF();
    /*** 
     * Tell me how many states and outputs you have!
     * @param n_states Number of the elements on the input vector x
     * @param n_outputs Number of the elements on the output vector z
     * @param Q Process noise covariance
     * @param R Measurement noise covariance
     */
    void InitSystem(int n_states, int n_outputs, const MatrixXd& Q, const MatrixXd& R);
    /*** 
     * Define model of your system.
     * @param x System states
     * @param u System inputs
     */
    virtual VectorXd f(const VectorXd &x, const VectorXd &u);
    /*** 
     * Define the output model of your system. 
     * @param x System states
     */
    virtual VectorXd h(const VectorXd &x);
    /*** 
     * Initialize the system states.Must be called after InitSystem.
     * If not called, system state is initialized to zero.
     * @param x0 Inital value for the system state
     */
    void InitSystemState(const VectorXd& x0);
    /*** 
     *Initialize the system state covariance.Must be called after InitSystem.
     *If not called, state covariance is initialized to an identity matrix.
     * @param P0 Inital value for the state covariance
     */
    void InitSystemStateCovariance(const MatrixXd& P0);
    /*** 
     * Do the extended Kalman iteration step-by-step while simulating the system. 
     * Simulating the system is done to calculate system states and outputs.
     * @param u The applied input to the system
     */
    void EKalmanf(const VectorXd& u);
    /*** 
     * Do the extended Kalman iteration step-by-step without simulating the system. 
     * Use this if measurement is available and simulating the system is unnecessary.
     * Here, true system states and system outputs do not matter. 
     * The only thing that matters is the estimated states. 
     * @param z The measurement outputs
     * @param u The applied input to the system
     * @return {*}
     */
    void EKalmanf(const VectorXd& z, const VectorXd& u);
    /*** 特殊函数!!!!!一般不使用
     * Get current simulated true state.在没有观测数据z时依照过程方程,高斯噪声模拟的状态变量
     * @return Current simulated state of the system x_ 
     */
    const VectorXd& GetCurrentState() const;
    /*** 特殊函数!!!!!一般不使用
     * Get current simulated true output.This is analogous to the measurements
     * @return Current simulated output z_ 在没有观测数据z时依照观测方程,高斯噪声模拟的观测变量
     */
    const VectorXd& GetCurrentOutput() const;
    /*** 
     * Get current estimated output. x_m_
     * @return Current estimated output x_m_
     */
    const VectorXd& GetCurrentEstimatedState() const;
    /*** 
     * Get current estimated output. z_m_
     * @return {*} Current estimated output z_m_
     */
    const VectorXd& GetCurrentEstimatedOutput() const;

    private:
void CalcF(const VectorXd &x, const VectorXd &u);
void CalcH(const VectorXd &x);
    MatrixXd K;//Gain
    MatrixXd F_;// Jacobian of F 
    MatrixXd H_;// jacobian of H
    MatrixXd Q_;// Process noise covariance
    MatrixXd R_;// Measurement noise covariance
/*     VectorXd sqrt_Q_;//Process noise stdev Q对角线元素开方，标准差
    VectorXd sqrt_R_;//Measurement noise stdev  R对角线元素开方，标准差 */
    MatrixXd P_p_;//State covariance after a priori update
    MatrixXd P_m_;//State covariance after measurement update
    VectorXd v_;// Gaussian process noise
    VectorXd w_;// Gaussian measurement noise
    VectorXd x_m_;//State vector after measurement update
    VectorXd x_p_;//State vector after a priori update
    VectorXd z_m_;//Estimated output  即h(x_m_);
    double epsilon_; // Very small number for Jacobian
    fstream log;//for data analysis

    //特殊数据!!!!!!!!!!!!!!!!!!!!!!!!!!!! 如果状态不可观,使用EKalmanf(const VectorXd &u)才有用
    VectorXd x_;//State vector 在没有观测数据z时依照过程方程,高斯噪声模拟的状态变量
    VectorXd z_;//Output matrix 在没有观测数据z时依照观测方程,高斯噪声模拟的观测变量
    //特殊数据!!!!!!!!!!!!!!!!!!!!!!!!!!!! 如果状态不可观,使用EKalmanf(const VectorXd &u)才有用
protected:
    int nStates_;   // Number of the states
    int nOutputs_;  // Number of outputs   
    long long int iteration;//
};  
}

#endif
