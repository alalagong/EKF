#include <iostream>
#include"IMU_ESKF.hpp"

using namespace std;
using namespace Eigen;

Matrix3d Lpms_IMU_ESKF::skew_symmetric(Vector3d &v)
{
	Matrix3d m;
	m << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return m;
}

 //四元数表示旋转时对四元数求导2
//diff_(qstar*v*q)/ diff_q = 2[qw*v - qv×v | -v*(qv)' + qv*(v)' + (qv)'*v*I + qw×v]
MatrixXd Lpms_IMU_ESKF::diff_qstarvq_q(Quaterniond &q, Vector3d &v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2 * (q0*v + skew_symmetric(v)*qv);
	D.block<3, 3>(0, 1) = 2 * (-v*qv.transpose() + qv*v.transpose()  + v.dot(qv)*Matrix3d::Identity() + q0*skew_symmetric(v));
	return D;
}

Vector3d Lpms_IMU_ESKF::mat2euler(Matrix3d m)
{
	double r =  atan2(-m(2, 1), m(2, 2));
	double p = asin( m(2, 0));
	double y = atan2(-m(1, 0), m(0, 0)) + Magnetic_declination;
	Vector3d rpy(r/deg2pi, p/deg2pi, y/deg2pi);
	return rpy;
}

//构造函数，初始化state,PQR
Lpms_IMU_ESKF::Lpms_IMU_ESKF()
{
    x_error = VectorXd::Zero(n_state);
    x_nominal = VectorXd::Zero(n_state +1);
    x_nominal(0) = 1;
    z_hat = VectorXd::Zero(6);
    current_t = 0;

    P = MatrixXd::Identity(n_state,n_state);
    R = MatrixXd::Identity(n_state,n_state);
    Q = MatrixXd::Identity(n_state,n_state);

    P.block<3,3>(0,0) = theta_cov*Matrix3d::Identity();
    P.block<3,3>(3,3) = wb_cov*Matrix3d::Identity();
    //观测值协方差
    R.block<3,3>(0,0) = acc_cov*Matrix3d::Identity();
    R.block<3,3>(3,3) = mag_cov*Matrix3d::Identity();

    Q.block<3,3>(0,0) = gyro_cov*Matrix3d::Identity();
    Q.block<3,3>(3,3) = gyro_bia_cov*Matrix3d::Identity();
    reset();
}
//析构函数
Lpms_IMU_ESKF::~Lpms_IMU_ESKF()
{
    // Removes the initialized sensor
    manager->removeSensor(lpms);
     // Deletes LpmsSensorManager object
    delete manager;
}
//重置函数，重置状态变量及状态误差协方差
void Lpms_IMU_ESKF::reset()
{
    initialized = false;
    IMU_initialized = false;
    euler_initialized = false;
    //reset x_error
    x_error = VectorXd::Zero(6);
    P = MatrixXd::Identity(6,6)*P*MatrixXd::Identity(6,6).transpose();
    init_lpmsIMU();//初始化IMU
    get_rawData();//获取原始数据
    init_eulerAngle();//初始化欧拉角
    initialized = true;
    cout<<"All initialization completed"<<endl;
}
//对IMU设备进行初始化
void Lpms_IMU_ESKF:: init_lpmsIMU()
{
    // Gets a LpmsSensorManager instance
    if(!IMU_initialized)
    {
        if(manager == nullptr && lpms == nullptr)
        {
            manager = LpmsSensorManagerFactory();
            lpms = manager->addSensor(DEVICE_LPMS_U2, "/dev/ttyUSB0");
        }

    if (lpms->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED&& !lpms->hasImuData())
        {
            cout<<"IMU is connecting . . . . . ."<<endl;
        }
        else{
            cout<<"IMU is initialized"<<endl;
            IMU_initialized = true;
        }
        this_thread::sleep_for(chrono::milliseconds(100));
    }

}
//得到IMU原始数据
void Lpms_IMU_ESKF::get_rawData()
 {
    ImuData d;
    if(IMU_initialized == true && lpms->hasImuData() && lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED)//获得IMU数据
    {
        d = lpms->getCurrentData();
         this_thread::sleep_for(chrono::milliseconds(1));
         //陀螺仪来计算角度的预测值
        raw_gyro[0] = d.gRaw[0]*deg2pi;//转化为弧度
        raw_gyro[1] = d.gRaw[1]*deg2pi;
        raw_gyro[2] = d.gRaw[2]*deg2pi;
        //他们用来矫正数据
        raw_acc[0] = d.aRaw[0];
        raw_acc[1] = d.aRaw[1];
        raw_acc[2] = d.aRaw[2];
        raw_mag[0] = d.bRaw[0];
        raw_mag[1] = d.bRaw[1];
        raw_mag[2] = d.bRaw[2];
//        cout<<"Time : "<<d.timeStamp<<"q.w : "<<d.q[0]<<"q.x : "<<d.q[1]<<"q.y : "<<d.q[2]<<"q.z : "<<d.q[3]<<endl;
//        cout<<"Time : "<<d.timeStamp<<"Accx : " <<raw_acc[0]<<"Accy : " <<raw_acc[1]<<"Accz : " <<raw_acc[2]<<endl;
//        cout<<"Time : "<<d.timeStamp<<"Gyrox : " <<raw_gyro[0]<<"Accy : " <<raw_gyro[1]<<"Accz : " <<raw_gyro[2]<<endl;
          cout<<"Time : "<<d.timeStamp<<"\tEulerAngle(deg) :  "<<d.r[0]<<"  " <<d.r[1]<<"  "<< d.r[2]<<endl;
        if(d.timeStamp > current_t)
        {
            dt = d.timeStamp - current_t;
            this->current_t = d.timeStamp;//更新当前时间
        }else{
            cout<<"time occurs error"<<endl;
        }
    }
    else{//没有初始化，开始初始化
       reset();
    }
 }

void Lpms_IMU_ESKF::init_eulerAngle()
{
    if(!euler_initialized && IMU_initialized )
    {
        //根据加速度计和磁力计求欧拉角
        ///rpy and passive , thought
        double roll = atan2(-raw_acc[1],-raw_acc[2]);
        double pitch = atan(raw_acc[0]/sqrt(raw_acc[2]*raw_acc[2]+raw_acc[1]*raw_acc[1]));//除以重力加速度
        double yaw =0 ;
         yaw = atan2( raw_mag[2]*sin(roll) - raw_mag[1]*cos(roll) , raw_mag[0]*cos(pitch) + raw_mag[1]*sin(pitch)*sin(roll) + raw_mag[2]*sin(pitch)*cos(roll)) - Magnetic_declination ;
        cout<<"Roll : "<<roll/deg2pi<<"pitch : "<<pitch/deg2pi<<"yaw : "<<yaw/deg2pi<<endl;
        AngleAxisd rollAngle(roll, Vector3d::UnitX());
        AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
        AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
        //转化为欧拉角并赋值给四元数
        Quaterniond q = yawAngle*pitchAngle*rollAngle; // n->b 矩阵之间的变换非向量
        q.normalize();
        x_nominal(0) = q.w();
        x_nominal.segment<3>(1) = q.vec();

//          cout<<"initialized Euler :"<<mat2euler(q.toRotationMatrix()) / deg2pi<<endl;
        euler_initialized = true;
        cout<<"Euler angle is initialized"<<endl;
    }

}
//返回当前的时间
double Lpms_IMU_ESKF::get_time()
{
    return current_t;
}


void Lpms_IMU_ESKF::predict()
{
    if(!initialized)
    {
        reset();
    }
    //更新数据，得到dt和加速度，磁力计数据
    get_rawData();

    //error值的角度和角速度零偏
    Vector3d wb_error = x_error.segment<3>(3);
    Vector3d wb_nominal = x_nominal.segment<3>(4);
    Vector3d wm = raw_gyro;//角速度测量值
    Quaterniond q_nominal ;
    q_nominal.w() = x_nominal(0);
    q_nominal.vec() = x_nominal.segment<3>(1);
    //系数矩阵
    Fx.setZero(n_state,n_state);
    Fi.setZero(n_state,n_state);

    //求nominal状态预测值
    Quaterniond w_q;
    w_q.w() = 1; w_q.vec() = 0.5*(wm - wb_nominal)*dt;
    q_nominal *= w_q;
    q_nominal.normalize();
    x_nominal(0) = q_nominal.w();
    x_nominal.segment<3>(1) = q_nominal.vec();


    //求R‘{(wm-wb)*dt},Rodrigues formula
    Vector3d omega = (wm - wb_error)*dt;
    double theta = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]); //转角
    Vector3d n = omega / theta; //转轴
    Matrix3d R = Matrix3d::Identity()*cos(theta) + (1 - cos(theta) )*n*n.transpose() + sin(theta)*skew_symmetric(n);

    //根据公式求系数矩阵
    Fx.block<3,3>(0,0) =R.transpose();
    Fx.block<3,3>(0,3) = - Matrix3d::Identity()*dt;
    Fx.block<3,3>(3,3) = Matrix3d::Identity();
    Fi = MatrixXd::Identity(6, 6);
    //求error状态预测值
    x_error = Fx*x_error;
    P = Fx*P*Fx.transpose() + Fi*Q*dt*Fi.transpose();
}
void Lpms_IMU_ESKF::measurement()
{
    Quaterniond q_nominal;
    q_nominal.w() = x_nominal(0);
    q_nominal.vec() = x_nominal.segment<3>(1);
    //加速度预测值
    Vector3d g_global(0,0,-1);
    z_hat.segment<3>(0) = q_nominal.inverse()*g_global;
    //磁力计预测值
    Vector3d mag_global = q_nominal*raw_mag;

    double norm = mag_global.segment<2>(0) .norm();
//    mag_global(0) = norm;
//    mag_global(1) =0;
    z_hat.segment <3>(3) = q_nominal.inverse()*mag_global;

    //求系数H矩阵
    H = MatrixXd::Zero(6,n_state);
    MatrixXd h1 = MatrixXd::Zero(6,7);
    h1.block<3,4>(0, 0) = diff_qstarvq_q(q_nominal, g_global);
    h1.block<3,4>(3, 0) = diff_qstarvq_q(q_nominal, mag_global);

    MatrixXd h2 = MatrixXd::Zero(7,6);
    double w, x, y, z;
    w = q_nominal.w();   x = q_nominal.x();  y = q_nominal.y();  z = q_nominal.z();
    Matrix<double,4,3> q_left;
    q_left << -x, -y, -z,
                    w, -z, y,
                    z, w, -x,
                    -y, x, w;
    h2.block<4,3>(0,0) = 0.5*q_left;
    h2.block<3,3>(4,3) = Matrix3d::Identity();
    H = h1*h2;

}
//误差模型求四元数标称值
void Lpms_IMU_ESKF::error2nominal()
{
    Quaterniond q_nominal;
    q_nominal.w() = x_nominal(0);
    q_nominal.vec() = x_nominal.segment<3>(1);
    //四元数更新
    Vector3d omega = x_error.segment<3>(0);
    cout<<"omega"<<endl<<omega<<endl;
    double omega_norm = omega.transpose()*omega;
    Quaterniond delt_q;
    delt_q.w() = cos(omega_norm/2);
    cout<<"  delt_q.w"<<  delt_q.w()<<endl;
    if(omega_norm == 0)
        delt_q.vec() = Vector3d::Zero();
    else
        delt_q.vec() = omega/omega_norm*sin(omega_norm/2);
    cout<<"delt_q.vec"<<endl<<delt_q.vec()<<endl;
    q_nominal = q_nominal*delt_q;
//    cout<<"q_nominal"<<q_nominal.w()<<q_nominal.vec()<<endl;
    q_nominal.normalize();
    x_nominal(0) = q_nominal.w();
    x_nominal.segment<3>(1) = q_nominal.vec();
    //wb更新
    x_nominal.segment<3>(4) += x_error.segment<3>(3);
}
//l利用测量值对状态进行修正，求出估计值
void Lpms_IMU_ESKF::estimate()
{
    //计算卡尔曼增益
    MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
    VectorXd z = VectorXd::Zero(6);
    z.segment<3>(0) = raw_acc;//已经更新的值做为测量
    z.segment<3>(3) = raw_mag;
//    cout<<"state :"<<x<<endl;
    cout<<"Z :"<<z<<endl;
    cout<<"Z_hat :"<<z_hat<<endl;
    cout<<"( z - z_hat)"<<endl<<( z - z_hat)<<endl;
    x_error = K*( z - z_hat);

    //计算新的协方差矩阵，留着下次更新用
    MatrixXd I = MatrixXd::Identity(n_state, n_state);
    P = (I - K*H)*P;
//	P = (I - K*H)*P*(I - K*H).transpose() + K*R*K.transpose();
    //误差加入到q中
    error2nominal();
    //reset x_error
    x_error = VectorXd::Zero(6);
    P = MatrixXd::Identity(6,6)*P* MatrixXd::Identity(6,6).transpose();
}

Observation Lpms_IMU_ESKF::get_Data()
{
    predict();
    measurement();
    estimate();

    Quaterniond q;
    q.w() = x_nominal(0);
    q.vec() = x_nominal.segment<3>(1);

    Observation data;
    data.Euler = mat2euler(q.toRotationMatrix());
    data.Quat = q;
    return data;
}

