/***
 * @Author: ztr
 * @Date: 2022-02-26 18:53:51
 * @LastEditTime: 2022-02-26 20:01:46
 * @LastEditors: ztr
 * @Description:
 * @FilePath: /EKF_KF_UKF/test/src/ekf_example2_RM.cpp
 */
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

#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include "ekf.hpp"

/// @cond DEV
/*
 * Class EKF needs to be derived, two virtual functions are provided in
 * which system model and output model are described.
 */
// X=[px,py] U=[l.x l.y yaw]
using namespace my_filter;
class MyEKF : public EKF
{
public:
  MyEKF(string logname="log.txt"):EKF(logname)
  {}
  virtual VectorXd f(const VectorXd &x, const VectorXd &u)
  {
    VectorXd pxpy_predict(nOutputs_); // 麦轮 运动学模型
    pxpy_predict(0) = x(0) + (u(0) * cos(u(2)) - u(1) * sin(u(2))) * 0.01;
    pxpy_predict(1) = x(1) + (u(0) * sin(u(2)) + u(1) * cos(u(2))) * 0.01;
    return pxpy_predict;
  }

  virtual VectorXd h(const VectorXd &x)
  {
    VectorXd px_py_measure(nOutputs_);
    px_py_measure(0) = x(0);
    px_py_measure(1) = x(1);
    return px_py_measure;
  }

private:
  /* static double dt=0.01; */
};
/// @endcond

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
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
  Q << 0.001, 0,
      0, 0.001;
  R << 0.1, 0,
      0, 0.1;
  VectorXd x0(2);
  x0 << 0.9, 0.9; //初始位置
  VectorXd u(3);


  MyEKF myekf("test.txt");
  myekf.InitSystem(n_states, n_outputs, Q, R);
  myekf.InitSystemState(x0);
#ifdef readdata
  ifstream infile;
  //参考http://cn.voidcc.com/question/p-vpcvenhz-zg.html
  infile.open("/home/ztr/workspace/my_repository/EKF_KF_UKF/data_analysis/posts_data.txt"); //将文件流对象与文件连接起来
  assert(infile.is_open());                                                                 //若失败,则输出错误消息,并终止程序运行
  int j;
  int i = 0;
  double lx, ly, yaw, px, py, px_noise, py_noise;
  while (infile >> j >> lx >> ly >> yaw >> px >> py >> px_noise >> py_noise && (i < 10))
  {
    i++;
    cout << "第" << j << "行数据如下" << endl;
    cout << "lx:  " << lx << "   ly:" << ly << "   yaw:" << yaw << endl;
    cout << "px:  " << px << "   py:" << py << "   px_noise:" << px_noise << "   py_noise" << py_noise << endl;
    cout << "下一行数据" << endl;
  }
#endif
ifstream infile;
 infile.open("/home/ztr/workspace/my_repository/EKF_KF_UKF/data_analysis/posts_data.txt"); //将文件流对象与文件连接起来
  assert(infile.is_open());                                                                 //若失败,则输出错误消息,并终止程序运行
  int j;
  int i = 0;
  double lx, ly, yaw, px, py, px_noise, py_noise;
  Vector2d z;//观测量
while (infile >> j >> lx >> ly >> yaw >> px >> py >> px_noise >> py_noise)
  { u<<lx,ly,yaw;//当前输入量
    z<<px_noise,py_noise;//当前观测量
    myekf.EKalmanf(z,u);
  }
//myekf.EKalmanf(z,u);

  /*  for (int k = 0; k < 1000; k ++) {
     u(0) = k;
     myekf.EKalmanf(u);
     VectorXd x(2),x_m(2),z(1);
     x = myekf.GetCurrentState();
     x_m = myekf.GetCurrentEstimatedState();
     z = myekf.GetCurrentOutput();
     //'Position Measurements', 'True Position', 'Position Estimates'
   } */
  return 0;
}
