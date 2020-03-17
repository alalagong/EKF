#ifndef IMU_EKF2_H_INCLUDED
#define IMU_EKF2_H_INCLUDED

#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<iostream>
#include <cstdio>
#include <thread>
#include<cmath>
#include <string>
#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"
#define REALTIME true
namespace IMU_EKF2
{

struct SensorData
{
    //IMU九轴的值
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Vector3d mag;
    Eigen::Vector3d groundtrue_euler;
};
class EKF
{
public:
    //是否使用Eigen库的内容
    enum UseLibrary{NO, YES} ;

    UseLibrary use_library_;
    //默认为读取数据集(0)，可选REALTIME(1)
    bool is_realtime_;

    EKF(UseLibrary options, bool isRealtime = false);
    ~EKF();

    bool runEKF();

private:

    //打开
    bool openIMU();
    bool openDatasets();
    //读数据
    bool getSensorDataFiles();
    bool getSensorDataIMU();
    bool getSensorData(bool isNormlize = true);
    //对加速度计和磁力计数据进行归一化
    void normalizeSensorData();
    //初始化
    void initializeStateByEuler();
    void initializeCovMatrix();
    //预测状态
    void predictStateVariableLibrary( Eigen::VectorXd &x_previous );
    void predictStateVariableCustom( Eigen::VectorXd  &x_previous );
    //计算状态转换矩阵
    void calTransitionMatrixLibrary( Eigen::VectorXd  &x_previous );
    void calTransitionMatrixCustom( Eigen::VectorXd  &x_previous );
    //预测协方差矩阵
    void predictCovMatrix();
    //得到测量值
    void getMeasurementsValue();
    //计算观测值
    Eigen::Vector3d estiamteObservationValueLibrary();
    Eigen::Matrix<double, 1, 4>  estimateObservationValueCustom();
    //计算观测矩阵
    void calObservationMatrixLibrary(Eigen::Vector3d mag_world);
    void calObservationMatrixCustom( Eigen::Matrix<double, 1, 4> b);
    //更正状态和协方差矩阵
    void correctStateVariable();
    //保存数据到文件
    void saveData(std::string interpretation);
    //欧拉角转换四元数(rpy)
    Eigen::Quaterniond euler2quatLibrary( Eigen::Vector3d euler );
    Eigen::Quaterniond euler2quatCustom( Eigen::Vector3d euler );
    //求导计算公式
    Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v);
    //diff_(p*q)/ diff_p
    Eigen::Matrix4d diff_pq_p(Eigen::Quaterniond q);
    //diff_(p*q) /diff_q
    Eigen::Matrix4d diff_pq_q(Eigen::Quaterniond p);
    //四元数表示旋转时对四元数求导2
    //diff_(qstar*v*q)/ diff_q = 2[qw*v - qv×v | -v*(qv)' + qv*(v)' + (qv)'*v*I + qw×v]
    Eigen::MatrixXd diff_qstarvq_q(Eigen::Quaterniond q, Eigen::Vector3d v);
    //四元数转欧拉角函数
    Eigen::Vector3d quat2euler( Eigen::Quaterniond &q);

    Eigen::VectorXd state_variable_;//状态变量
    Eigen::VectorXd observation_value_;//观测值
    Eigen::VectorXd measurement_value_; //测量值
    SensorData sensor_data_;

    Eigen::MatrixXd K_; //卡尔曼增益

    Eigen::MatrixXd P_;//状态协方差

    Eigen::MatrixXd F_;//状态方程系数矩阵
    Eigen::MatrixXd H_;//观测方程系数矩阵

    Eigen::MatrixXd G_;//状态噪声协方差系数
    Eigen::MatrixXd Q_;//状态噪声协方差矩阵

    Eigen::MatrixXd R_;//观测噪声协方差矩阵
    //常量
    const double WN_VAR_ = 8e-5;  //角速度噪声协方差
    const double WBN_VAR_ = 8e-5;  //加速度偏差协方差
    const double AN_VAR_ = 8e-4;  //加速度协方差
    const double MN_VAR_ = 1e-2;   //磁力计协方差
    const double MAG_DEC = -(7.+44./60)/180*M_PI;//沈阳本地磁偏角为-7°44′
    const int STATE_NUMBER_ = 7;
    double DEG2RAD = M_PI/180;
    const Eigen::Vector3d g_WORLD = Eigen::Vector3d( 0, 0, -1);

    //读取IMU数据所用
    LpmsSensorManagerI* manager_ = nullptr;
    LpmsSensorI* lpms_ = nullptr;
    //读取datasets数据
    const std::string PATH2DATA_ = "/home/gong/Datasets/nav_data/NAV_2.txt";
    std::ifstream infile_;

    bool initialized_;
    double dt_;


};



}




#endif // IMU_EKF2_H_INCLUDED
