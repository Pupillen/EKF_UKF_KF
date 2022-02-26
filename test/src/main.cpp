/*** 
 * @Author: ztr
 * @Date: 2022-02-26 13:34:34
 * @LastEditTime: 2022-02-26 13:51:08
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /EKF_KF_UKF/test/main.cpp
 */
#include <iostream>
#include <random>
#include <eigen3/Eigen/Core>
using namespace std;
using namespace Eigen;
int main(int argc,char* argv[])
{        Vector3d r;
        random_device rd;
        default_random_engine rng(rd());
        normal_distribution<double> q(0,1);
        
        for (size_t i = 0; i < 3; i++)
        {
            cout<<q(rng)<<endl;
            r(i)=q(rng);
        }
        cout<<r<<endl;
       
    return 0;
}