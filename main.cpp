#include <iostream>
#include "IMU_EKF2.h"

using namespace std;

int main()
{
    IMU_EKF2::EKF ekf(IMU_EKF2::EKF::NO,  false);
    ekf.runEKF();
    return 0;
}
