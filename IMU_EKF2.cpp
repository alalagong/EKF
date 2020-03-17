#include "IMU_EKF2.h"
#define USB_ID   "/dev/ttyUSB0"
namespace IMU_EKF2
{
EKF::EKF(UseLibrary options, bool isRealtime) : use_library_(options), is_realtime_(isRealtime)
{
    state_variable_ = Eigen::VectorXd::Zero( EKF::STATE_NUMBER_ );
    observation_value_ = Eigen::VectorXd::Zero( 6 );
    measurement_value_ = Eigen::VectorXd::Zero(6);
}

EKF::~EKF()
{
    if( EKF::is_realtime_ == true)
    {
        // Removes the initialized sensor
        EKF::manager_->removeSensor(lpms_);
        // Deletes LpmsSensorManager object
        delete manager_;
    }
    else
    {
        EKF::infile_.close();
    }
}

bool EKF::openIMU()
{
        // Gets a LpmsSensorManager instance
        if(manager_ == nullptr && lpms_ == nullptr)
        {
            manager_ = LpmsSensorManagerFactory();
            lpms_ = manager_->addSensor(DEVICE_LPMS_U2, USB_ID);
        }

        if (lpms_->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED&& !lpms_->hasImuData())
        {
            std::cout<<"IMU is connecting . . . . . ."<<std::endl;
            EKF::initialized_ = false;
        }
        else
        {
            std::cout<<"IMU is initialized"<<std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if(lpms_->hasImuData() && lpms_->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED)
    {
        EKF::initialized_ = true;
        return true;
    }
    return false;
}

bool EKF::openDatasets()
{
    infile_.open(PATH2DATA_);

    if( infile_.is_open() )
    {
        EKF::initialized_ = true;
        return true;
    }
   else
   {
        std::cout<<"open failed"<<std::endl;
        return false;
   }
}

bool EKF::getSensorDataIMU()
{
    if( initialized_ == false || !lpms_->hasImuData() || lpms_->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED)
    {
        if( openIMU() == false )
            return false;
    }
    ImuData d;
    d = lpms_->getCurrentData();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EKF::sensor_data_.acc[0] = d.aRaw[0];
    EKF::sensor_data_.acc[1] = d.aRaw[1];
    EKF::sensor_data_.acc[2] = d.aRaw[2];
    EKF::sensor_data_.gyro[0] = d.gRaw[0]*DEG2RAD;//转化为弧度
    EKF::sensor_data_.gyro[1] = d.gRaw[1]*DEG2RAD;
    EKF::sensor_data_.gyro[2] = d.gRaw[2]*DEG2RAD;
    EKF::sensor_data_.mag[0] = d.bRaw[0];
    EKF::sensor_data_.mag[1] = d.bRaw[1];
    EKF::sensor_data_.mag[2] = d.bRaw[2];
    EKF::sensor_data_.groundtrue_euler[0] = d.r[0]*DEG2RAD;
    EKF::sensor_data_.groundtrue_euler[1] = d.r[1]*DEG2RAD;
    EKF::sensor_data_.groundtrue_euler[2] = d.r[2]*DEG2RAD;
    std::cout<<"groundtrue_Euler: "<<sensor_data_.groundtrue_euler.transpose()<<std::endl;
    dt_ = 0.01;
    return true;
}

bool EKF::getSensorDataFiles()
{
    if( initialized_ == false || !EKF::infile_.is_open() )
     {
        if( openDatasets() == false)
            return false;
     }
     //读取数据失败
     if(infile_.eof())
        return false;
    const int data_numbers = 36;
    double data_temp[data_numbers] = {0};
    for(int i = 0; i < data_numbers ; i++ )
    {
        infile_>>data_temp[i];
    }
    EKF::sensor_data_.acc[0] = data_temp[8];
    EKF::sensor_data_.acc[1] = data_temp[9];
    EKF::sensor_data_.acc[2] = data_temp[10];
//    EKF::sensor_data_.gyro[0] = data_temp[11];
//    EKF::sensor_data_.gyro[1] = data_temp[12];
//    EKF::sensor_data_.gyro[2] = data_temp[13];
    //师兄使用的，和上面的相同
    EKF::sensor_data_.gyro[0] = data_temp[26];
    EKF::sensor_data_.gyro[1] = data_temp[27];
    EKF::sensor_data_.gyro[2] = data_temp[28];
    EKF::sensor_data_.mag[0] = data_temp[14];
    EKF::sensor_data_.mag[1] = data_temp[15];
    EKF::sensor_data_.mag[2] = data_temp[16];
    EKF::sensor_data_.groundtrue_euler[0] = data_temp[29];
    EKF::sensor_data_.groundtrue_euler[1] = data_temp[30];
    EKF::sensor_data_.groundtrue_euler[2] = data_temp[31];
    dt_ = 0.02;
    //std::cout<<"groundtrue_Euler: "<<sensor_data_.groundtrue_euler.transpose()<<std::endl;
    return true;
}

void EKF::normalizeSensorData()
{

//    EKF::sensor_data_.acc.normalize();
//    EKF::sensor_data_.mag.normalize();

    double norm = std::sqrt(sensor_data_.acc[0]*sensor_data_.acc[0] + sensor_data_.acc[1]*sensor_data_.acc[1] + sensor_data_.acc[2]*sensor_data_.acc[2]);
    sensor_data_.acc[0] /= norm;
    sensor_data_.acc[1] /= norm;
    sensor_data_.acc[2] /= norm;
    norm = std::sqrt(sensor_data_.mag[0]*sensor_data_.mag[0] + sensor_data_.mag[1]*sensor_data_.mag[1] + sensor_data_.mag[2]*sensor_data_.mag[2]);
    sensor_data_.mag[0] /= norm;
    sensor_data_.mag[1] /= norm;
    sensor_data_.mag[2] /= norm;

}

bool EKF::getSensorData(bool isNormlize)
{
    bool isSuccess = false;
    if(is_realtime_)
        isSuccess = getSensorDataIMU();
    else
        isSuccess = getSensorDataFiles();
    //成功读取并且归一化
    if(isSuccess && isNormlize)
        normalizeSensorData();

    return isSuccess;

}

Eigen::Quaterniond EKF::euler2quatLibrary(Eigen::Vector3d euler)
{
    Eigen::AngleAxisd rollAngle(euler[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(euler[2], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond  q =yawAngle*pitchAngle*rollAngle;
    return q;
}

Eigen::Quaterniond EKF::euler2quatCustom(Eigen::Vector3d euler)
{
    double cYaw = cos(euler[2] / 2), sYaw = sin(euler[2] / 2);
	double cPit = cos(euler[1] / 2), sPit = sin(euler[1] / 2);
	double cRol = cos(euler[0] / 2), sRol = sin(euler[0] / 2);
	Eigen::Quaterniond q;
	q.w() = cRol*cPit*cYaw + sRol*sPit*sYaw;
	q.x() = sRol*cPit*cYaw - cRol*sPit*sYaw;
	q.y() = cRol*sPit*cYaw + sRol*cPit*sYaw;
	q.z() = cRol*cPit*sYaw - sRol*sPit*cYaw;
	return q;
}
//求一个向量的反对称矩阵[]×
Eigen::Matrix3d EKF::skew_symmetric(Eigen::Vector3d v)
{
	Eigen::Matrix3d m;
	m << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return m;
}
//四元数乘法的左乘矩阵[]L（叉积求导）
//diff_(p*q) /diff_q
Eigen::Matrix4d EKF::diff_pq_q(Eigen::Quaterniond p)
{
	double p0 = p.w();
	Eigen::Vector3d pv = p.vec();

	Eigen::Matrix4d D;
	D(0, 0) = p0;
	D.block<1, 3>(0, 1) = -pv.transpose();
	D.block<3, 1>(1, 0) = pv;
	D.block<3, 3>(1, 1) = Eigen::Matrix3d::Identity()*p0 + skew_symmetric(pv);
	return D;
}
//四元数乘法的右乘矩阵[]R（叉积求导）
//diff_(p*q)/ diff_p
Eigen::Matrix4d EKF::diff_pq_p(Eigen::Quaterniond q)
{
	double q0 = q.w();
	Eigen::Vector3d qv = q.vec();
	Eigen::Matrix4d D;
	D(0, 0) = q0;
	D.block<1, 3>(0, 1) = -qv.transpose();
	D.block<3, 1>(1, 0) = qv;
	D.block<3, 3>(1, 1) = Eigen::Matrix3d::Identity()*q0 - skew_symmetric(qv);
	return D;
}

Eigen::Vector3d EKF::quat2euler(Eigen::Quaterniond &q)
{
    double yaw, pitch, roll;
	double r1 = 2*(q.w()*q.x() + q.y()*q.z());
	double r2 = 1 - 2*(q.x()*q.x() + q.y()*q.y());
	double r3 = 2*(q.w()*q.y() - q.z()*q.x());
	double r4 = 2*(q.w()*q.z() + q.x()*q.y());
	double r5 = 1 - 2*(q.y()*q.y() + q.z()*q.z());

	roll = atan2(r1, r2);
	pitch = asin(r3);
	yaw = atan2(r4,r5) + MAG_DEC;

	Eigen::Vector3d euler(roll,pitch,yaw);

	return euler;
}

Eigen::MatrixXd EKF::diff_qstarvq_q(Eigen::Quaterniond q, Eigen::Vector3d v)
{
    double q0 = q.w();
	Eigen::Vector3d qv = q.vec();
	Eigen::MatrixXd D(3, 4);
	D.col(0) = 2 * (q0*v + skew_symmetric(v)*qv);
	D.block<3, 3>(0, 1) = 2 * (-v*qv.transpose() + qv*v.transpose()  + v.dot(qv)*Eigen::Matrix3d::Identity() + q0*skew_symmetric(v));
	return D;
}

void EKF::initializeStateByEuler()
{
        double roll = atan2(-sensor_data_.acc[1], -sensor_data_.acc[2]);
        //double pitch = asin(sensor_data_.acc[0]);//除以重力加速度
        double pitch = atan2(sensor_data_.acc[0], sqrt(sensor_data_.acc[1]*sensor_data_.acc[1] +sensor_data_.acc[2]*sensor_data_.acc[2]));
        double yaw = 0;
        yaw = atan2( sensor_data_.mag[2]*sin(roll) - sensor_data_.mag[1]*cos(roll) , sensor_data_.mag[0]*cos(pitch) + sensor_data_.mag[1]*sin(pitch)*sin(roll) + sensor_data_.mag[2]*sin(pitch)*cos(roll)) ;
        Eigen::Vector3d euler( roll, pitch, yaw );
        Eigen::Quaterniond q = EKF::euler2quatLibrary(euler);
        //Eigen::Quaterniond q = EKF::euler2quatCustom(euler);
        q.normalize();
        state_variable_(0) = q.w();
        state_variable_.segment<3>(1) = q.vec();
}

void EKF::initializeCovMatrix()
{
    P_ = Eigen::MatrixXd::Identity(EKF::STATE_NUMBER_, EKF::STATE_NUMBER_);//协方差矩阵

	Q_ = Eigen::MatrixXd::Identity(7, 7);//噪声的初始化
	Q_.block<4, 4>(0, 0) *= WN_VAR_;
	Q_.block<3, 3>(4, 4) *= WBN_VAR_;

    R_ = Eigen::MatrixXd::Identity(6,6);
    R_.block<3,3>(0,0) *= AN_VAR_;
    R_.block<3,3>(3,3) *= MN_VAR_;
}

void EKF::getMeasurementsValue()
{
    measurement_value_.segment<3>(0) = sensor_data_.acc;
    measurement_value_.segment<3>(3) = sensor_data_.mag;
}

void EKF::predictStateVariableLibrary( Eigen::VectorXd &x_previous )
{
    Eigen::VectorXd x_dot = Eigen::VectorXd::Zero( STATE_NUMBER_ );
    Eigen::Quaterniond q;
    q.w() = x_previous[0]; q.vec() = x_previous.segment<3>(1);
    Eigen::Quaterniond gyro_q(0, 0, 0, 0);
	gyro_q.vec() = sensor_data_.gyro - x_previous.segment<3>(4);
	Eigen::Quaterniond q_dot = q*gyro_q;//under local
	q_dot.w() /= 2; q_dot.vec() /= 2;//* and / to scalar is not support for Quaternion
	x_dot(0) = q_dot.w(); x_dot.segment<3>(1) = q_dot.vec();

    state_variable_ += x_dot*dt_;
    state_variable_.head(4).normalize();
}

void EKF::predictStateVariableCustom( Eigen::VectorXd &x_previous )
{
    Eigen::Vector3d gyro_correct;
    gyro_correct = sensor_data_.gyro - x_previous.segment<3>(4);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    x = x_previous.segment<4>(0);
    //update
	state_variable_[0] = x[0] + (-x[1]*gyro_correct[0] - x[2]*gyro_correct[1] - x[3]*gyro_correct[2]) * dt_/2;
	state_variable_[1] = x[1] + ( x[0]*gyro_correct[0] + x[2]*gyro_correct[2] - x[3]*gyro_correct[1]) * dt_/2;
	state_variable_[2] = x[2] + ( x[0]*gyro_correct[1] - x[1]*gyro_correct[2] + x[3]*gyro_correct[0]) * dt_/2;
	state_variable_[3] = x[3] + ( x[0]*gyro_correct[2] + x[1]*gyro_correct[1] - x[2]*gyro_correct[0]) * dt_/2;
    state_variable_.segment<3>(4) = x_previous.segment<3>(4);

	double norm;
	norm = sqrt(state_variable_[0]*state_variable_[0] + state_variable_[1]*state_variable_[1] + state_variable_[2]*state_variable_[2] + state_variable_[3]*state_variable_[3]);
	state_variable_[0] /= norm;
	state_variable_[1] /= norm;
	state_variable_[2] /= norm;
	state_variable_[3] /= norm;
}

void EKF::calTransitionMatrixLibrary(Eigen::VectorXd &x_previous)
{
    Eigen::Quaterniond q;
    q.w() = x_previous[0]; q.vec() = x_previous.segment<3>(1);
    F_.setZero(STATE_NUMBER_, STATE_NUMBER_);
    Eigen::Quaterniond gyro_q(0, 0, 0, 0);
	gyro_q.vec() = sensor_data_.gyro;
    F_.block<4, 4>(0, 0) = 0.5*diff_pq_p(gyro_q);
    F_.block<4, 3>(0, 4) = -0.5*(diff_pq_q(q).block<4, 3>(0, 1));
    F_ = Eigen::MatrixXd::Identity(STATE_NUMBER_, STATE_NUMBER_) + F_*dt_;
}

void EKF::calTransitionMatrixCustom(Eigen::VectorXd& x_previous)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    x = x_previous.segment<4>(0);
    F_.setZero( STATE_NUMBER_, STATE_NUMBER_ );
    F_.block<4, 4>(0, 0) << 0, -sensor_data_.gyro[0], -sensor_data_.gyro[1], -sensor_data_.gyro[2],
                                           sensor_data_.gyro[0], 0, sensor_data_.gyro[2], -sensor_data_.gyro[1],
                                           sensor_data_.gyro[1], -sensor_data_.gyro[2], 0, sensor_data_.gyro[0],
                                           sensor_data_.gyro[2], sensor_data_.gyro[1], -sensor_data_.gyro[0], 0;
    F_.block<4, 3>(0, 4) <<   x[1],  x[2],  x[3],
                                            -x[0],  x[3], -x[2],
                                            -x[3], -x[0],  x[1],
                                             x[2], -x[1], -x[0];
      F_ = Eigen::MatrixXd::Identity(STATE_NUMBER_, STATE_NUMBER_) +0.5* F_*dt_;
}

void EKF::predictCovMatrix()
{
    P_ = F_*P_*F_.transpose() + Q_;
}

Eigen::Vector3d EKF::estiamteObservationValueLibrary()
{
    Eigen::Quaterniond q;
    q.w() = state_variable_(0); q.vec() = state_variable_.segment<3>(1);
    //重力计
    observation_value_.segment <3>(0) = q.inverse()*g_WORLD;

    Eigen::Vector3d mag_world = q*sensor_data_.mag; //这里不同！！*****
    double norm = mag_world.segment<2>(0) .norm();
    mag_world(0) =norm;
    mag_world(1) =0;
    observation_value_.segment <3>(3) = q.inverse()*mag_world;
    return mag_world;
}

 Eigen::Matrix<double, 1, 4> EKF::estimateObservationValueCustom()
{
    double w, x, y, z;
    w =  state_variable_(0);
    x = state_variable_(1);
    y =  state_variable_(2);
    z =  state_variable_(3);
    observation_value_.segment<3>(0) << -2*(x*z - w*y),
                                                                  -2*(y*z + w*x),
                                                                  - 1 + 2*( x*x + y*y);
    Eigen::Vector3d mag_world ;
    //和师兄的不同
    mag_world << (1 - 2*(y*y + z*z))*sensor_data_.mag[0] + 2*(x*y-w*z)*sensor_data_.mag[1] + 2*(x*z+w*y)*sensor_data_.mag[2],
                                                     2*(x*y+w*z)*sensor_data_.mag[0] + (1-2*(z*z+x*x))*sensor_data_.mag[1] +2*(y*z-x*w)*sensor_data_.mag[2],
                                                     2*(x*z-y*w)*sensor_data_.mag[0] + 2*(y*z+x*w)*sensor_data_.mag[1] + (1-2*(x*x+y*y))*sensor_data_.mag[2];
//    mag_world << (1 - 2*(y*y + z*z))*sensor_data_.mag[0] + 2*(x*y+w*z)*sensor_data_.mag[1] + 2*(x*z-w*y)*sensor_data_.mag[2],
//                                                     2*(x*y-w*z)*sensor_data_.mag[0] + (1-2*(z*z+x*x))*sensor_data_.mag[1] +2*(y*z+x*w)*sensor_data_.mag[2],
//                                                     2*(x*z+y*w)*sensor_data_.mag[0] + 2*(y*z-x*w)*sensor_data_.mag[1] + (1-2*(x*x+y*y))*sensor_data_.mag[2];
    Eigen::Matrix<double, 1, 4> b;
    b[0] = 0;
	b[1] = sqrt(mag_world[0]*mag_world[0] + mag_world[1]*mag_world[1]);
	b[2] = 0;
	b[3] = mag_world[2];

    observation_value_.segment<3>(3) << b[1]*(1-2*(y*y+z*z)) + 2*b[3]*(x*z-w*y),
                                                                  2*b[1]*(x*y-w*z) + 2*b[3]*(w*x+y*z),
                                                                  2*b[1]*(x*z+y*w) + b[3]*(1-2*(x*x+y*y));
    return b;
}

void EKF::calObservationMatrixLibrary(Eigen::Vector3d mag_world)
{
    Eigen::Quaterniond q;
    q.w() = state_variable_(0); q.vec() = state_variable_.segment<3>(1);
    H_.setZero(STATE_NUMBER_-1, STATE_NUMBER_);
    H_.block<3,4>(0,0) = diff_qstarvq_q(q, g_WORLD);
    H_.block<3,4>(3,0) = diff_qstarvq_q(q, mag_world);
}

void EKF::calObservationMatrixCustom( Eigen::Matrix<double, 1, 4> b)
{
    double w, x, y, z;
    w =  state_variable_(0);
    x = state_variable_(1);
    y =  state_variable_(2);
    z =  state_variable_(3);
    H_.setZero(STATE_NUMBER_-1, STATE_NUMBER_);
    H_.block<3,4>(0,0) << y, -z, w, -x,
                                         -x, -w, -z, -y,
                                         -w, x, y, -z;

    H_.block<3,4>(3,0) << -b[3]*y, b[3]*z, -2*y*b[1]-b[3]*w, -2*b[1]*z+b[3]*x,
                                         -b[1]*z+b[3]*x, b[1]*y+b[3]*w, b[1]*x+b[3]*z, -b[1]*w+b[3]*y,
                                         b[1]*y, b[1]*z-2*b[3]*x, 2*b[1]*w - 2*b[3]*y, b[1]*x;
    H_ = 2*H_;
}

void EKF::correctStateVariable()
{
    K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();
    state_variable_ += K_*(measurement_value_ - observation_value_);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_NUMBER_, STATE_NUMBER_);
    P_ = (I - K_*H_)*P_;
    state_variable_.head(4).normalize();
}

bool EKF::runEKF()
{
    int index = 1;
    bool flag_norm = true;
    bool flag_open = false;
    std::string interpretation;
    Eigen::Quaterniond q;
    Eigen::Vector3d euler;
    Eigen::VectorXd state_last;
    //选择使用IMU还是数据集
    if(is_realtime_)
        flag_open = openIMU();
    else
        flag_open = openDatasets();
    //打开失败
    if(!flag_open)
    {
        std::cout<<"open failed"<<std::endl;
        return flag_open;
    }
    interpretation = "初始化数据";
    //获取数据
    getSensorData(flag_norm);
    //对状态变量和进行初始化
    initializeStateByEuler();
    initializeCovMatrix();
    //开始进行循环
    saveData(interpretation);
    while(getSensorData())
    {
        interpretation = "运算数据"+std::to_string(index);
        state_last = state_variable_;
        //得到测量值
        getMeasurementsValue();
        if(use_library_)
        {
            predictStateVariableLibrary(state_last);
            calTransitionMatrixLibrary(state_last);
            predictCovMatrix();
            calObservationMatrixLibrary(estiamteObservationValueLibrary());
            correctStateVariable();
        }
        else
        {
            predictStateVariableCustom(state_last);
            calTransitionMatrixCustom(state_last);
            predictCovMatrix();
            calObservationMatrixCustom(estimateObservationValueCustom());
            correctStateVariable();
        }
//        q.w() = state_variable_[0]; q.vec() = state_variable_.segment<3>(1);
//        euler = quat2euler(q);
        //euler = euler / DEG2RAD;
        saveData(interpretation);
        index++;
//        std::cout<<"my Euler :"<<euler.transpose() / DEG2RAD<<std::endl;
//        std::cout<<"truth Euler:"<<sensor_data_.groundtrue_euler.transpose()/DEG2RAD<<std::endl;
    }
    return true;
}

void EKF::saveData(std::string interpretation)
{
    Eigen::Vector3d euler;
    Eigen::Quaterniond q;
    q.w() = state_variable_[0]; q.vec() = state_variable_.segment<3>(1);
    euler = quat2euler(q);
    std::ofstream outfile;
    outfile.open("out.txt", std::ios::app);
    if( !outfile )
    {
        std::cout<<"save data failed "<< std::endl;
        return;
    }

    outfile<< interpretation;
    outfile<< "\n";
    outfile<<"truth: ";
    outfile<< sensor_data_.groundtrue_euler.transpose() / DEG2RAD ;
    outfile<<"\n";
    outfile<<"euler: ";
    outfile<<euler.transpose() /DEG2RAD;
    outfile<<"\n";
    outfile.close();
}

}
