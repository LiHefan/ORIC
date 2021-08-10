#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "so3.h"
#include "NavState.h"
#include "IMUPreintegrator.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

namespace g2o
{
using namespace ORB_SLAM2;

// class VertexIDP
// {

// };

class VertexNavStatePR: public BaseVertex<6,NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexNavStatePR(): BaseVertex<6,NavState>() {}

    bool read(std::istream& is) {return true;}
    
    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl(){
        _estimate = NavState();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<const Vector6d> update(update_);
        _estimate.IncSmallPR(update);
    }

};

class VertexNavStateV: public BaseVertex<3, NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexNavStateV(): BaseVertex<3,NavState>(){}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl(){
        _estimate = NavState();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<const Vector3d> update(update_);
        _estimate.IncSmallV(update);
    }
};

class EdgeNavStatePRV: public BaseMultiEdge<9,IMUPreintegrator>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePRV() : BaseMultiEdge<9,IMUPreintegrator>(){
        resize(5);
    }

    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os) const {return true;}

    void computeError();

    virtual void linearizeOplus();

    void SetParams(const Vector3d& gw){
        GravityVec = gw;
    }
protected:
    // Gravity vector in 'world' frame
    Vector3d GravityVec;
};

class EdgeNavStatePRPointXYZ: public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePRPointXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePR>() {}

    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os) const {return true;}

    void computeError(){
        Vector3d Pc=computePc();
        Vector2d obs(_measurement);

        _error = obs - cam_project(Pc);
    }

    bool isDepthPositive(){
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

    Vector3d computePc(){
        const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexNavStatePR* vNavState = static_cast<const VertexNavStatePR*>(_vertices[1]);

        const NavState& ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        return Pc;
    }
    inline Vector2d project2d(const Vector3d& v) const {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }
    Vector2d cam_project(const Vector3d& trans_xyz) const{
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    virtual void linearizeOplus();

    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_){
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        Rbc = Rbc_;
        Pbc = Pbc_;
    }

protected:
    // camera intrinsics
    double fx, fy, cx, cy;
    // camera-imu extrinsic
    Matrix3d Rbc;
    Vector3d Pbc;
};

class EdgeNavStatePRPointXYZOnlyPose: public BaseUnaryEdge<2, Vector2d, VertexNavStatePR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeNavStatePRPointXYZOnlyPose(){}

    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os) const {return true;}
    
    void computeError(){
        Vector3d Pc = computePc();
        Vector2d obs(_measurement);

        _error = obs - cam_project(Pc);
    }

    bool isDepthPositive(){
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

    Vector3d computePc(){
        const VertexNavStatePR* vNSPR = static_cast<const VertexNavStatePR*>(_vertices[0]);
        const NavState& ns = vNSPR->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb *Rwb.transpose()*(Pw - Pwb) - Rcb * Pbc;

        return Pc;
    }

    inline Vector2d project2d(const Vector3d& v) const{
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Vector2d cam_project(const Vector3d& trans_xyz) const{
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    virtual void linearizeOplus();

    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_, const Vector3d& Pw_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        Rbc = Rbc_;
        Pbc = Pbc_;
        Pw = Pw_;
    }

    protected:
        double fx, fy, cx, cy;
        Matrix3d Rbc;
        Vector3d Pbc;
        Vector3d Pw;

};

/*
 Vertex: PR, V, Bias
 Error: dP, dV, dBg, dBa
*/
class EdgeNavStatePriorPRVBias: public BaseMultiEdge<15,NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePriorPRVBias() : BaseMultiEdge<15,NavState>() {resize(3);}

    bool read(std::istream &is){return true;}
    bool write(std::ostream &os) const{return true;}

    void computeError();
    virtual void linearizeOplus();
};

/**
 * @brief The VertexNavStatePVR class
*/
class VertexNavStatePVR: public BaseVertex<9,NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexNavStatePVR(): BaseVertex<9, NavState>(){}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl(){
        _estimate = NavState();
    }

    virtual void oplusImpl(const double* update_){
        Eigen::Map<const Vector9d> update(update_);
        _estimate.IncSmallPVR(update);
    }
};

class VertexNavStateBias: public BaseVertex<6,NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexNavStateBias() : BaseVertex<6,NavState>(){}

    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl(){
        _estimate = NavState();
    }

    virtual void oplusImpl(const double* update_){
        Eigen::Map<const Vector6d> update(update_);
        _estimate.IncSmallBias(update);
    }

};

class EdgeNavStatePVR : public BaseMultiEdge<9,IMUPreintegrator>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePVR() : BaseMultiEdge<9, IMUPreintegrator>(){
        resize(3);
    }

    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os ) {return true;}

    void computeError();

    virtual void linearizeOplus();

    void SetParams(const Vector3d& gw){
        GravityVec = gw;
    }

protected:
    Vector3d GravityVec;
};

class EdgeNavStateBias : public BaseBinaryEdge<6, IMUPreintegrator, VertexNavStateBias, VertexNavStateBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeNavStateBias() : BaseBinaryEdge<6, IMUPreintegrator, VertexNavStateBias, VertexNavStateBias>() {}
    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os) const {return true;}

    void computeError();
    virtual void linearizeOplus();

};

class EdgeNavStatePVRPointXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePVR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePVRPointXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePVR>() {}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    void computeError() {
        Vector3d Pc = computePc();
        Vector2d obs(_measurement);

        _error = obs - cam_project(Pc);
    }

    bool isDepthPositive(){
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

     Vector3d computePc(){
        const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexNavStatePVR* vNavState = static_cast<const VertexNavStatePVR*>(_vertices[1]);

        const NavState& ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb *Rwb.transpose()*(Pw - Pwb) - Rcb * Pbc;

        return Pc;
    }

    inline Vector2d project2d(const Vector3d& v) const{
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Vector2d cam_project(const Vector3d& trans_xyz) const{
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    virtual void linearizeOplus();

    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        Rbc = Rbc_;
        Pbc = Pbc_;
    }

    protected:
        double fx, fy, cx, cy;
        Matrix3d Rbc;
        Vector3d Pbc;    
};

class  EdgeNavStatePVRPointXYZOnlyPose : public BaseUnaryEdge<2, Vector2d, VertexNavStatePVR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeNavStatePVRPointXYZOnlyPose(){}

    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os) const {return true;}

    void computeError() {
        Vector3d Pc = computePc();
        Vector2d obs(_measurement);

        _error = obs - cam_project(Pc);
    }

    bool isDepthPositive(){
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

     Vector3d computePc(){
        const VertexNavStatePVR* vNSPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);

        const NavState& ns = vNSPVR->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb *Rwb.transpose()*(Pw - Pwb) - Rcb * Pbc;

        return Pc;
    }

    inline Vector2d project2d(const Vector3d& v) const{
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Vector2d cam_project(const Vector3d& trans_xyz) const{
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    virtual void linearizeOplus();

    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_, const Vector3d& Pw_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        Rbc = Rbc_;
        Pbc = Pbc_;
        Pw = Pw_;
    }

    protected:
        double fx, fy, cx, cy;
        Matrix3d Rbc;
        Vector3d Pbc;
        Vector3d Pw;
};

class EdgeNavStatePriorPVRBias : public BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePriorPVRBias() : BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias>() {}

    bool read(std::istream &is){return true;}
    bool write(std::ostream &os) const{return true;}

    void computeError();

    virtual void linearizeOplus(); 
};

//---------------------------------------------------------------

/**
 * @brief The VertexNavState class
 * Vertex of tightly-coupled Visual-Inertial optimization
 */
class VertexNavState : public BaseVertex<15,NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexNavState();
    
    bool read(std::istream& is);
    bool write(std::ostream& os) const;

    virtual void setToOriginImpl(){
        _estimate = NavState();
    }

    virtual void oplusImpl(const double* update_);

};

/**
 * @brief The EdgeNavStatePrior class
 */

class EdgeNavStatePrior : public BaseUnaryEdge<15, NavState, VertexNavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePrior(){}
    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os) const {return true;}

    void computeError();

    virtual void linearizeOplus();
};

/**
 * @brief The EdgeNavState class
 * Edge between NavState_i and NavState_j, vertex[0]~i, vertex[1]~j
 * Measurement~Vector15d: 9Dof-IMUPreintegrator measurement  & 6Dof-IMU bias change all Zero
 */

class EdgeNavState : public BaseBinaryEdge<15, IMUPreintegrator, VertexNavState, VertexNavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavState();

    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    
    void computeError();

    virtual void linearizeOplus();

    void SetParams(const Vector3d& gw){
        GravityVec = gw;
    }
protected:
    // Gravity vector in 'world' frame
    Vector3d GravityVec;
};

/**
 * @brief The EdgeNavStatePointXYZ class
 * Edge between NavState and Point3D, vertex[0]~Point3D, vertex[1]~NavState
 * Measurement~Vector2d: 2DOf image feature position
 */
class EdgeNavStatePointXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePointXYZ();

    bool read(std::istream& is);
    
    bool write(std::ostream& os) const;

    void computeError() {
        Vector3d Pc = computePc();
        Vector2d obs(_measurement);

        _error = obs - cam_project(Pc);
    }

    bool isDepthPositive() {
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

    Vector3d computePc() {
        const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexNavState* vNavState = static_cast<const VertexNavState*>(_vertices[1]);

        const NavState& ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        return Pc;
        //Vector3d Pwc = Rwb*Pbc + Pwb;
        //Matrix3d Rcw = (Rwb*Rbc).transpose();
        //Vector3d Pcw = -Rcw*Pwc;
        //Vector3d Pc = Rcw*Pw + Pcw;
    }
    inline Vector2d project2d(const Vector3d& v) const {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }
    Vector2d cam_project(const Vector3d & trans_xyz) const {
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    //
    virtual void linearizeOplus();

    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        Rbc = Rbc_;
        Pbc = Pbc_;
    }

protected:
    // Camera intrinsics
    double fx, fy, cx, cy;
    // Camera-IMU extrinsics
    Matrix3d Rbc;
    Vector3d Pbc;
};

class EdgeNavStatePointXYZOnlyPose : public BaseUnaryEdge<2, Vector2d, VertexNavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePointXYZOnlyPose(){}

    bool read(std::istream& is) {return true;}
    bool write(std::ostream& os) const {return true;}

    void computeError() {
        Vector3d Pc = computePc();
        Vector2d obs(_measurement);

        _error = obs - cam_project(Pc);
    }

    bool isDepthPositive() {
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

    Vector3d computePc() {
        const VertexNavState* vNavState = static_cast<const VertexNavState*>(_vertices[0]);

        const NavState& ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        //const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        return Pc;
        //Vector3d Pwc = Rwb*Pbc + Pwb;
        //Matrix3d Rcw = (Rwb*Rbc).transpose();
        //Vector3d Pcw = -Rcw*Pwc;
        //Vector3d Pc = Rcw*Pw + Pcw;
    }
    inline Vector2d project2d(const Vector3d& v) const {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }
    Vector2d cam_project(const Vector3d & trans_xyz) const {
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    //
    virtual void linearizeOplus();

    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_, const Vector3d& Pw_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        Rbc = Rbc_;
        Pbc = Pbc_;
        Pw = Pw_;
    }

protected:
    // Camera intrinsics
    double fx, fy, cx, cy;
    // Camera-IMU extrinsics
    Matrix3d Rbc;
    Vector3d Pbc;
    // Point position in world frame
    Vector3d Pw;
};

/** 
 * @brief The VertexGyrBias class
 * For gyroscope bias computation in Visual-Inertial initialization
 */
class VertexGyrBias : public BaseVertex<3, Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexGyrBias();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    virtual void setToOriginImpl(){
        _estimate.setZero();
    }

    virtual void oplusImpl(const double* update_);

};

/**
 * @brief The EdgeGyrBias class
 * For gyroscope bias computation in Visual-Inertial initialization
 */
class EdgeGyrBias: public BaseUnaryEdge<3, Vector3d, VertexGyrBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyrBias();

    bool read(std::istream& is);
    bool write(std::ostream& os) const;

    Matrix3d dRbij;
    Matrix3d J_dR_bg;
    Matrix3d Rwbi;
    Matrix3d Rwbj;

    void computeError();

    virtual void linearizeOplus();

};

}
#endif