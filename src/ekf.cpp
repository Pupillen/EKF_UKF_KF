/***
 * @Author: ztr
 * @Date: 2022-02-25 23:35:17
 * @LastEditTime: 2022-02-26 11:42:14
 * @LastEditors: ztr
 * @Description:
 * @FilePath: /EKF_KF_UKF/src/ekf.cpp
 */
#include <random>
#include "ekf.hpp"
#include <iostream>
#include <eigen3/Eigen/Core>
#include <assert.h>
#include <iomanip>
namespace my_filter
{
    using namespace Eigen;
    using namespace std;
    EKF::EKF()
    {
        log.open("../log_file.txt");
        iteration = 0;
    }

    EKF::~EKF()
    {
        log.close();
        iteration = 0;
    }

    //假定Q，R是对角矩阵
    void EKF::InitSystem(int n_states, int n_outputs, const MatrixXd &Q, const MatrixXd &R)
    {
        assert(Q.cols() == Q.rows() && "Q must be a square matrix");
        assert(R.cols() == R.rows() && "R must be a square matrix");
        Q_ = Q;
        R_ = R;
        epsilon_ = 1e-8; // Epsilon for computing the Jacobian numerically
        nStates_ = n_states;
        nOutputs_ = n_outputs;
        K.resize(nOutputs_, n_states);
        x_.resize(n_states);
        x_p_.resize(n_states);
        x_m_.resize(n_states);
        F_.resize(n_states, n_states);  //偏f偏x
        H_.resize(n_outputs, n_states); //偏h偏x
        P_p_.resize(n_states, n_states);
        P_m_.resize(n_states, n_states);
        v_.resize(n_states);
        w_.resize(n_outputs);
        P_m_.setIdentity(); // Inital values for the Kalman iterations
        x_.setZero();       // Apply intial states
        x_m_.setZero();     // Apply intial states
    }

    VectorXd EKF::f(const VectorXd &x, const VectorXd &u) // need to be override
    {
        VectorXd X_k(nStates_);
        X_k.setZero();
        return X_k;
    }

    VectorXd EKF::h(const VectorXd &x)
    {
        VectorXd Z_k(nOutputs_);
        Z_k.setZero();
        return Z_k;
    }

    void EKF::CalcF(const VectorXd &x, const VectorXd &u)
    {
        VectorXd f0(nStates_, nStates_);
        VectorXd fn(nStates_, nStates_);
        f0 = f(x, u);
        /* 求偏f偏x,即f的第i个分量对x的第j个分量求导，即xj微小偏移后求f的第i个分量的差分*/
        for (int j = 0; j < nStates_; j++)
        {
            VectorXd X_eps(x);
            X_eps(j) += epsilon_; // xj微小偏移后
            fn = f(X_eps, u);     //导致f偏移
            for (int i = 0; i < nStates_; i++)
            {
                F_(i, j) = (fn(i) - f0(i)) / epsilon_; // f的第i个分量的差分
            }
        }
#ifdef Debug
        for (int i = 0; i < nStates_; i++)
        {
            for (int j = 0; j < nStates_; j++)
            {
                cout << "F为如下数值" << endl;
                cout.setf(ios::fixed);
                cout << setw(9) << setprecision(3) << F_(i, j);
                if (j == (nStates_ - 1))
                    cout << endl;
            }
        }

#endif
    }
    void EKF::CalcH(const VectorXd &x)
    {
        VectorXd h0(nOutputs_, nOutputs_);
        VectorXd hn(nOutputs_, nOutputs_);
        h0 = h(x);
        /* 求偏h偏x,即h的第i个分量对x的第j个分量求导，即xj微小偏移后求h的第i个分量的差分*/
        for (int j = 0; j < nStates_; j++)
        {
            VectorXd X_eps(x);
            X_eps(j) += epsilon_; // xj微小偏移后
            hn = h(X_eps);        //导致f偏移
            for (int i = 0; i < nOutputs_; i++)
            {
                H_(i, j) = (hn(i) - h0(i)) / epsilon_; // f的第i个分量的差分
            }
        }
#ifdef Debug
        for (int i = 0; i < nOutputs_; i++)
        {
            for (int j = 0; j < nStates_; j++)
            {
                cout << "H为如下数值" << endl;
                cout.setf(ios::fixed);
                cout << setw(9) << setprecision(3) << H_(i, j);
                if (j == (nStates_ - 1))
                    cout << endl;
            }
        }

#endif
    }
    void EKF::InitSystemState(const VectorXd &x0)
    {
        x_ = x0;
        x_m_ = x0;
    }
    void EKF::InitSystemStateCovariance(const MatrixXd &P0)
    {
        P_m_ = P0;
    }
    void EKF::EKalmanf(const VectorXd &u)
    {
        iteration++;
        random_device rd;
        default_random_engine rng(rd());
        normal_distribution<double> q(0, 1);
        normal_distribution<double> r(0, 1);
        for (int i = 0; i < nStates_; i++)
        {
            v_(i) = sqrt(Q_(i, i)) * r(rng);
        }
        for (int i = 0; i < nOutputs_; i++)
        {
            w_(i) = sqrt(R_(i, i)) * r(rng);
        }
        x_ = f(x_, u) + v_;
        z_ = h(x_) + w_;
        //模拟系统状态变量x_与z_
        CalcF(x_m_, u); //计算在上一次最优估计下的F矩阵；
        // prior update
        x_p_ = f(x_m_, u);
        P_p_ = F_ * P_m_ * F_.transpose() + Q_;
        CalcH(x_p_); //计算在本次已经估计的情况下的H矩阵
        // Measurement update
        K = P_p_ * H_.transpose() * (H_ * P_p_ * H_.transpose() + R_).inverse();
        x_m_ = x_p_ + K * (z_ - h(x_p_)); //没有观测变量z,所以只能使用前面方程模拟的z_
        P_m_ = P_p_ - K * H_ * P_p_;
    }
    void EKF::EKalmanf(const VectorXd &z, const VectorXd &u)
    {
        iteration++;
        CalcF(x_m_, u); //计算在上一次最优估计下的F矩阵；
        // prior update
        x_p_ = f(x_m_, u);
        P_p_ = F_ * P_m_ * F_.transpose() + Q_;
        CalcH(x_p_); //计算在本次已经估计的情况下的H矩阵
        // Measurement update
        K = P_p_ * H_.transpose() * (H_ * P_p_ * H_.transpose() + R_).inverse();
        x_m_ = x_p_ + K * (z - h(x_p_));
        P_m_ = P_p_ - K * H_ * P_p_;
        log << iteration;
        //观测数据
        for (int i = 0; i < nOutputs_; i++)
        {

            log << "\t" << z(i);
        }
        // EKF后观测数据
        for (int i = 0; i < nOutputs_; i++)
        {

            log << "\t" << z_m_(i);
        }
        //状态变量数据
        for (int i = 0; i < nStates_; i++)
        {

            log << "\t" << z_m_(i);
            if (i == (nStates_ - 1))
                log << "\n";
        }
    }

    const VectorXd &EKF::GetCurrentState() const
    {
        return x_;
    }
    const VectorXd &EKF::GetCurrentOutput() const
    {
        return z_;
    }
    const VectorXd &EKF::GetCurrentEstimatedState() const
    {
        return x_m_;
    }
    const VectorXd &EKF::GetCurrentEstimatedOutput() const
    {
        return z_m_;
    }
}
