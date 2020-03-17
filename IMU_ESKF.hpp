#ifndef IMU_ESKF_HPP_INCLUDED
#define IMU_ESKF_HPP_INCLUDED
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<iostream>
#include <cstdio>
#include <thread>
#include<cmath>
#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"
using namespace std;
using namespace Eigen;
#define deg2pi (M_PI/180)
///state for kalman filter
/*
    0-2     thetax thetay thetaz    角度的误差量
    3-5     w_bx w_by w_bz          陀螺仪偏差(deg/s)
*/
///Input
/*
    0-2   wx wy wz                 三轴加速度输入(g = 9.8m/s^2)
    3-5   ax ay az                   三轴角速度输入(deg/s)
    6-8   magx magy magz    三轴磁力计
*/
///observation
/*
    0-2 ax ay az                     三轴加速度值(g)
    3-5 magx magy magz      三轴磁力计值(uT)
*/
///inertial frame : ENU
///local -> global
///Right hand
///passive

    struct state
	{
        Vector3d theta;
        Vector3d wb;
	};
    struct Observation
    {
        Quaterniond Quat;
        Vector3d Euler;
    };
class Lpms_IMU_ESKF
{
public:
    Lpms_IMU_ESKF();
    ~Lpms_IMU_ESKF();

    void reset();//对参数进行复位
    void predict();
    void error2nominal();//误差值到标称值转换
    void measurement();//根据当前状态获得观测值
    void estimate();//得到状态估计值
    Observation get_Data();//返回相应的值
    double get_time();//得到当前时间
    Quaterniond raw_q;
    bool initialized;
private:
    VectorXd x_error;//误差状态变量
    VectorXd x_nominal;//标称值状态
    VectorXd z_hat;//观测值
    MatrixXd P;//状态协方差

    MatrixXd Fx;//状态方程系数矩阵
    MatrixXd H;//观测方程系数矩阵

    MatrixXd Fi;//状态噪声协方差系数
    MatrixXd Q;//状态噪声协方差矩阵

    MatrixXd R;//观测噪声协方差矩阵

    const double gyro_cov = 8e-5;//陀螺仪偏差协方差
    const double gyro_bia_cov = 7e-6;

    const double acc_cov = 8e-4;//加速度计偏差协方差
	const double mag_cov = 1e-4;
//初始化协方差
    const double theta_cov = 1e-5;
    const double wb_cov = 1e-7;

    const int n_state = 6;//状态数量
    const double Magnetic_declination = -(7+44/60)/180*M_PI;//沈阳本地磁偏角为-7°44′， 这个值是咋回事？？
    LpmsSensorManagerI* manager = nullptr;
    LpmsSensorI* lpms = nullptr;
    //IMU九轴的值
    Vector3d raw_acc;
    Vector3d raw_gyro;
    Vector3d raw_mag;

    //is initialized
    bool IMU_initialized;
    bool euler_initialized;

    double current_t;//当前时间
    double dt;//时间差
    void init_lpmsIMU();//对IMU进行配置
    void init_eulerAngle();//使用加速度计和磁力计进行欧拉角的初始化
    void get_rawData();//得到原IMU始数据
    Vector3d mat2euler(Matrix3d m);
    MatrixXd diff_qstarvq_q(Quaterniond &q, Vector3d &v);
    Matrix3d skew_symmetric(Vector3d &v);
};
#endif // IMU_ESKF_HPP_INCLUDED
