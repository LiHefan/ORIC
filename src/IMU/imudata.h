#ifndef IMUDATA_H
#define IMUDATA_H

#include <Eigen/Dense>
namespace ORB_SLAM2
{
using namespace Eigen;
class IMUData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // covariance of measurement
    static Matrix3d _gyrMeasCov;
    static Matrix3d _accMeasCov;
    static Matrix3d getGyrMeasCov(void) {return _gyrMeasCov;}
    static Matrix3d getAccMeasCov(void) {return _accMeasCov;}

    // covariance of bias random walk
    static Matrix3d _gyrBiasRWCov;
    static Matrix3d _accBiasRWCov;
    static Matrix3d getGyrBiasRWCov(void) {return _gyrBiasRWCov;}
    static Matrix3d getAccBiasRWCov(void) {return _accBiasRWCov;}

    static double _gyrBiasRW2;
    static double _accBiasRW2;
    static double getGyrBiasRW2(void) {return _gyrBiasRW2;}
    static double getAccBiasRW2(void) {return _accBiasRW2;}

    IMUData(const double &gx, const double &gy, const double &gz,
            const double &ax, const double &ay, const double &az,
            const double &t):_g(gx,gy,gz), _a(ax,ay,az),_t(t)
    {
    }
    Vector3d _g;
    Vector3d _a;
    double _t;
};

}

#endif