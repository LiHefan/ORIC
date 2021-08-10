#include "NavState.h"

namespace ORB_SLAM2
{
NavState::NavState()
{
    _P.setZero();
    _V.setZero();
    Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
    _R = Sophus::SO3(I3x3);

    _BiasGyr.setZero();
    _BiasAcc.setZero();

    _dBias_g.setZero();
    _dBias_a.setZero();
}

NavState::NavState(const NavState &_ns):
    _P(_ns._P),_V(_ns._V),_R(_ns._R),
    _BiasGyr(_ns._BiasGyr), _BiasAcc(_ns._BiasAcc),
    _dBias_g(_ns._dBias_g), _dBias_a(_ns._dBias_a)
{
    
}

void NavState::IncSmall(Vector15d update)
{
    Vector3d upd_P = update.segment<3>(0);
    Vector3d upd_V = update.segment<3>(3);
    Vector3d upd_Phi = update.segment<3>(6);
    Vector3d upd_dBg = update.segment<3>(9);
    Vector3d upd_dBa = update.segment<3>(12);

    _P += upd_P;
    _V += upd_V;
    Sophus::SO3 dR = Sophus::SO3::exp(upd_Phi);
    _R = Get_R()*dR;
    _dBias_g += upd_dBg;
    _dBias_a += upd_dBa;

}

void NavState::IncSmallPR(Vector6d dPR)
{
    Vector3d upd_P = dPR.segment<3>(0);
    Vector3d upd_Phi = dPR.segment<3>(3);

    _P += upd_P;
    _R = _R * Sophus::SO3::exp(upd_Phi);
}

void NavState::IncSmallV(Vector3d dV)
{
    _V += dV;
}

void NavState::IncSmallPVR(Vector9d updatePVR)
{
    Vector3d upd_P = updatePVR.segment<3>(0);
    Vector3d upd_V = updatePVR.segment<3>(3);
    Vector3d upd_Phi = updatePVR.segment<3>(6);

    Matrix3d R = Get_R().matrix();

    _P += upd_P;
    _V += upd_V;
    Sophus::SO3 dR = Sophus::SO3::exp(upd_Phi);
    _R = Get_R()*dR;
}

void NavState::IncSmallBias(Vector6d updatedBias)
{
    Vector3d upd_dBg = updatedBias.segment<3>(0);
    Vector3d upd_dBa = updatedBias.segment<3>(3);

    _dBias_g += upd_dBg;
    _dBias_a += upd_dBa;
}


}