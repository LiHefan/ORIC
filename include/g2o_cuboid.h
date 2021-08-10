#pragma once

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "matrix_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>  //std::swap
#include <string>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/intersection.hpp> 

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace g2o{
//立方体类cuboid
class cuboid{
public:
    SE3Quat pose; //6自由度位姿，object to world
    Eigen::Vector3d scale; //3自由度尺度[length,width,height] half!
    std::string name; 

    cuboid(){
        pose=SE3Quat();
        scale.setZero();
        name="";
    }

    //分别记录下9个自由度位姿参数
        //xyz rpy lwh(half)
    inline void fromMinimalVector(const Vector9d& v){
        Eigen::Quaterniond posequat= zyx_euler_to_quat(v(3),v(4),v(5));
        pose=SE3Quat(posequat,v.head<3>());
        scale=v.tail<3>();
    }
        //xyz quaternion lwh(half)
    inline void fromVector(const Vector10d&v){
        pose.fromVector(v.head<7>());
        scale=v.tail<3>();
    }

    inline const Eigen::Vector3d& translation() const {return pose.translation();}
    inline void setTranslation(const Eigen::Vector3d& t_){pose.setTranslation(t_);}
    inline void setRotation(const Eigen::Quaterniond& r_) {pose.setRotation(r_);}
    inline void setRotation(const Eigen::Matrix3d& R) {pose.setRotation(Eigen::Quaterniond(R));}
    inline void setScale(const Eigen::Vector3d &scale_){scale=scale_;}

    //更新cuboid //apply update to current cuboid, exponential map
    cuboid exp_update(const Vector9d& update){
        cuboid res;
        res.pose=this->pose*SE3Quat::exp(update.head<6>());
        res.scale=this->scale+update.tail<3>();
        return res;
    }
    
    //用于计算camera-object误差,见论文IV-B公式(8)//actual error between two cuboids 
    Vector9d cube_log_error(const cuboid& newone) const{
        Vector9d res;
        SE3Quat pose_diff=newone.pose.inverse()*this->pose;
        res.head<6>()=pose_diff.log();
        res.tail<3>()=this->scale-newone.scale;
        return res;
    }

    //function called by g2o
    Vector9d min_log_error(const cuboid& newone, bool print_details=false) const{
        bool whether_rotate_cubes=true;     //whether rotate cube to find smallest error 
        if(!whether_rotate_cubes)
            return cube_log_error(newone);
        
        //正反面交换(0度和180度yaw)、长宽交换(+-90度yaw以及l,w交换)导致不同cuboid参数可能表示相同立方体，进行误差计算时应考虑此问题,即取各情况中最小误差 
        Eigen::Vector4d rotate_errors_norm;
        Eigen::Vector4d rotate_angles(-1,0,1,2); //rotate -90 0 90 180
        Eigen::Matrix<double,9,4> rotate_errors;
        for(int i=0;i<rotate_errors_norm.rows();++i){
            cuboid rotated_cuboid =newone.rotate_cuboid(rotate_angles(i)*M_PI/2.0); //rotate new cuboids
            Vector9d cuboid_error=this->cube_log_error(rotated_cuboid);
            rotate_errors_norm(i)=cuboid_error.norm();
            rotate_errors.col(i)=cuboid_error;
        }

        int min_label;
        rotate_errors_norm.minCoeff(&min_label);
        if(print_details)
            if(min_label!=1)
                std::cout<<"Rotate cube "<<min_label<<std::endl;
        return rotate_errors.col(min_label);

    }

    
    //change front face by rotate along current body z axis. another way of representing cuboid. representing same cuboid(IOU always 1)
    cuboid rotate_cuboid(double yaw_angle) const{   //to deal with different front surface of cuboids
        cuboid res;
        SE3Quat rot(Eigen::Quaterniond(cos(yaw_angle*0.5),0,0,sin(yaw_angle*0.5)), Eigen::Vector3d(0,0,0)); //change yaw to rotation
        res.pose=this->pose*rot;
        res.scale=this->scale;
        if( (yaw_angle==M_PI/2.0)||(yaw_angle==-M_PI/2.0)||(yaw_angle==3*M_PI/2.0) )
            std::swap(res.scale(0), res.scale(1));

        return res;
    } 

    //transform a local cuboid to global cuboid. Twc is camera pose,from camera to world
    cuboid transform_from(const SE3Quat& Twc) const{
        cuboid res;
        res.pose=Twc*this->pose;
        res.scale=this->scale;
        return res;
    }

    //transform a global cuboid to local cuboid. Twc is camera pose,from camera to world
    cuboid transform_to(const SE3Quat& Twc) const{
        cuboid res;
        res.pose=Twc.inverse()*this->pose;
        res.scale=this->scale;
        return res;

    }

    //将位姿信息写成向量
        //xyz rpy lwh(half)
    inline Vector9d toMinimalVector() const{
        Vector9d v;
        v.head<3>()=pose.translation();
        Eigen::Quaterniond r=pose.rotation();
        double roll,pitch,yaw;
        quat_to_euler_zyx(r,roll,pitch,yaw);
        v[3]=roll;
        v[4]=pitch;
        v[5]=yaw;
        v.tail<3>()=scale;
        return v;

    }
        //xyz quaternion lwh(half)
    inline Vector10d toVector() const{
        Vector10d v;
        v.head<7>()=pose.toVector();
        v.tail<3>()=scale;
        return v;

    }

    //物体位姿信息转换,pose+scale->Matrix4d res
    Eigen::Matrix4d similarityTransform() const{
        Eigen::Matrix4d res=pose.to_homogeneous_matrix();
        Eigen::Matrix3d scale_mat=scale.asDiagonal();
        res.topLeftCorner<3,3>()=res.topLeftCorner<3,3>()*scale_mat;
        return res;
    }

    //将九自由度的位姿信息转换成８个点的坐标
    //对于８个角，定义３＊８的矩阵表示其坐标corners_body
        //8corners 3*8 matrix,each row is x y z
    Eigen::Matrix3Xd compute3D_BoxCorner() const{
        Eigen::Matrix3Xd corners_body;
        corners_body.resize(3,8);
        corners_body<< 1,  1, -1, -1, 1,  1, -1, -1,
					   1, -1, -1,  1, 1, -1, -1,  1,
					  -1, -1, -1, -1, 1,  1,  1,  1;
        //similarityTransform包含4*4的位姿
        //corners_body包含3*8的顶点坐标
        //real_to_homo_coord将3*8转换成4*8
        //homo_to_real_coord将4*8转换成3*8
        Eigen::Matrix3Xd corners_world=homo_to_real_coord<double>(similarityTransform()*real_to_homo_coord<double>(corners_body));

        return corners_world;
    } 

    //cuboid重投影成矩形框　
        //get rectangles after projection [topleft bottomright]
    Eigen::Vector4d projectOntoImageRect(const SE3Quat& campose_cw, const Eigen::Matrix3d& Kalib) const{
        Eigen::Matrix3Xd corners_3d_world = compute3D_BoxCorner();
        Eigen::Matrix2Xd corner_2d=homo_to_real_coord<double>( Kalib*            
                                    homo_to_real_coord<double>( campose_cw.to_homogeneous_matrix() * real_to_homo_coord<double>(corners_3d_world) ) );
        Eigen::Vector2d bottomright=corner_2d.rowwise().maxCoeff();
        Eigen::Vector2d topleft=corner_2d.rowwise().minCoeff();
        return Eigen::Vector4d(topleft(0),topleft(1),bottomright(0),bottomright(1));

    }

        //get rectangles after projection [center width height]
    Eigen::Vector4d projectOntoImageBbox(const SE3Quat& campose_cw,const Eigen::Matrix3d& Kalib) const{
        Eigen::Vector4d rect_project=projectOntoImageRect(campose_cw,Kalib);
        Eigen::Vector2d rect_center=(rect_project.tail<2>()+rect_project.head<2>())/2;
        Eigen::Vector2d widthheight=rect_project.tail<2>()-rect_project.head<2>();

        return Eigen::Vector4d(rect_center(0),rect_center(1),widthheight(0),widthheight(1));
    }

    double box3d_iou(cuboid sample, cuboid ground)
    {
        // get 2d area in the top
        // append outer and inners() https://www.boost.org/doc/libs/1_65_1/libs/geometry/doc/html/geometry/reference/models/model_polygon.html
        typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
        typedef boost::geometry::model::polygon<point_t> polygon_t;
        polygon_t sam_poly, gt_poly;
        Eigen::Matrix<double, 3, 8> sam_corner_3d = sample.compute3D_BoxCorner();
        Eigen::Matrix<double, 3, 8> gt_corner_3d = ground.compute3D_BoxCorner();
        for (size_t i = 0; i < 4; i++)
        {
        point_t sam_top_points(sam_corner_3d(0,i),sam_corner_3d(1,i));
        point_t gt_top_points(gt_corner_3d(0,i),gt_corner_3d(1,i));
        boost::geometry::append(sam_poly.outer(), sam_top_points);
        boost::geometry::append(gt_poly.outer(), gt_top_points);
        if (i == 3) // add start point to make a closed form
        {
            boost::geometry::append(sam_poly.outer(), point_t(sam_corner_3d(0,0),sam_corner_3d(1,0)));
            boost::geometry::append(gt_poly.outer(), point_t(gt_corner_3d(0,0),gt_corner_3d(1,0)));    
        }
        }
        std::vector<polygon_t> inter_poly;
        boost::geometry::intersection(sam_poly, gt_poly, inter_poly); 
        double inter_area = inter_poly.empty() ? 0 : boost::geometry::area(inter_poly.front());
        double union_area = boost::geometry::area(sam_poly) + boost::geometry::area(gt_poly) - inter_area;// boost::geometry::union_(poly1, poly2, un);
        double iou_2d = inter_area / union_area;
        // std::cout << "iou2d: " << iou_2d << std::endl;

        double h_up = std::min(sam_corner_3d(2,4),gt_corner_3d(2,4));
        double h_down = std::max(sam_corner_3d(2,0),gt_corner_3d(2,0));
        double inter_vol = inter_area * std::max(0.0, h_up - h_down);
        double sam_vol = sample.scale(0)*2 * sample.scale(1)*2 * sample.scale(2)*2;
        double gt_vol = ground.scale(0)*2 * ground.scale(1)*2 * ground.scale(2)*2;
        double iou_3d = inter_vol / (sam_vol + gt_vol - inter_vol);
        // std::cout << "iou3d: " << iou_3d << std::endl;
        return iou_3d;
    }
};

class VertexCuboid:public BaseVertex<9,cuboid>{     //this vertex stores object pose to world
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCuboid(){};

    virtual void setToOriginImpl(){_estimate=cuboid();}
    virtual void oplusImpl(const double* update_){
        Eigen::Map<const Vector9d> update(update_); //Eigen::Map接受Ｃ++中普通数组/指针来构造Matrix(Vector)
        setEstimate(_estimate.exp_update(update));  
    }

    virtual bool read(std::istream& is){
        Vector9d est;
        for(int i=0;i<9;++i)
            is>>est[i];
        cuboid Onecube;
        Onecube.fromMinimalVector(est);
        setEstimate(Onecube);
        return true;
    }

    virtual bool write(std::ostream& os) const{
        Vector9d lv=_estimate.toMinimalVector();
        for(int i=0;i<lv.rows();++i){
            os<<lv[i]<<" ";
        }
        return os.good();
    }
};

// camera-object 3Derror
class EdgeSE3Cuboid:public BaseBinaryEdge<9,cuboid,VertexSE3Expmap,VertexCuboid>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3Cuboid(){}

    virtual bool read(std::istream& is){
        return true;
    }

    virtual bool write(std::ostream& os) const{
        return os.good();
    }

    void computeError(){
        const VertexSE3Expmap* SE3Vertex=static_cast<const VertexSE3Expmap*>(_vertices[0]); //world to camera pose
        const VertexCuboid* cuboidVertex=static_cast<const VertexCuboid*>(_vertices[1]);    //object pose to world

        SE3Quat cam_pose_Twc=SE3Vertex->estimate().inverse();
        cuboid global_cube=cuboidVertex->estimate();
        cuboid esti_global_cube=_measurement.transform_from(cam_pose_Twc);
        _error=global_cube.min_log_error(esti_global_cube);
    }
};

class EdgeSE3CuboidIOU:public BaseBinaryEdge<9,cuboid,VertexSE3Expmap,VertexCuboid>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3CuboidIOU(){}

    virtual bool read(std::istream& is){
        return true;
    }

    virtual bool write(std::ostream& os) const{
        return os.good();
    }

    void computeError(){
        const VertexSE3Expmap* SE3Vertex=static_cast<const VertexSE3Expmap*>(_vertices[0]);
        const VertexCuboid* cuboidVertex=static_cast<const VertexCuboid*>(_vertices[1]);

        SE3Quat cam_pose_Twc=SE3Vertex->estimate().inverse();
        cuboid global_cube=cuboidVertex->estimate();
        cuboid esti_global_cube=_measurement.transform_from(cam_pose_Twc);
        double iou = global_cube.box3d_iou(global_cube,esti_global_cube);
        Vector9d error;
        error << 1,1,1,1,1,1,1,1,1;
        error = error * (1-iou);
        _error=error;
    }
};

// camera-object 2D projection error, rectangle difference, could also change to IOU
class EdgeSE3CuboidProj:public BaseBinaryEdge<4,Eigen::Vector4d,VertexSE3Expmap,VertexCuboid>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3CuboidProj(){}

    virtual bool read(std::istream& is){
        return true;
    }

    virtual bool write(std::ostream& os) const{
        return os.good();
    }

    void computeError(){
        const VertexSE3Expmap* SE3Vertex=static_cast<const VertexSE3Expmap*>(_vertices[0]); //world to camera pose
        const VertexCuboid* cuboidVertex=static_cast<const VertexCuboid*>(_vertices[1]);    //object pose to world

        SE3Quat cam_pose_Tcw=SE3Vertex->estimate();
        cuboid global_cube=cuboidVertex->estimate();

        Eigen::Vector4d rect_project=global_cube.projectOntoImageBbox(cam_pose_Tcw,Kalib); //center, width, height

        _error = rect_project-_measurement; 
    }

    Eigen::Matrix3d Kalib;
};


}


