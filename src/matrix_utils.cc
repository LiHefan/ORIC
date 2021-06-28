#include "matrix_utils.h"

#include <math.h>
#include <stdio.h>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <ctime>

using namespace Eigen;

template <class T>
Eigen::Quaternion<T> zyx_euler_to_quat(const T&roll, const T&pitch,const T&yaw){
    T sy=sin(yaw*0.5);
    T cy=cos(yaw*0.5);
    T sp=sin(pitch*0.5);
    T cp=cos(pitch*0.5);
    T sr=sin(roll*0.5);
    T cr=cos(roll*0.5);
    T w=cr*cp*cy+sr*sp*sy;
    T x=sr*cp*cy-cr*sp*sy;
    T y=cr*sp*cy+sr*cp*sy;
    T z=cr*cp*sy-sr*sp*cy;

    return Eigen::Quaternion<T>(w,x,y,z);

}
template Eigen::Quaterniond zyx_euler_to_quat<double>(const double&, const double&,const double&);
template Eigen::Quaternionf zyx_euler_to_quat<float>(const float&, const float&,const float&);

template <class T>
void quat_to_euler_zyx(const Eigen::Quaternion<T>& q, T& roll, T& pitch, T& yaw){
    const T qw=q.w();
    const T qx=q.x();
    const T qy=q.y();
    const T qz=q.z();

    roll = atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy));
	pitch = asin(2*(qw*qy-qz*qx));
	yaw = atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz));
}
template void quat_to_euler_zyx<double>(const Eigen::Quaterniond&, double&, double&, double&);
template void quat_to_euler_zyx<float>(const Eigen::Quaternionf&, float&, float&, float&);

//输入：pts_in 3*8
//输出：pts_homo_out 4*8
template <class T>
Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic> real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_in){
    Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic> pts_homo_out;
    int raw_rows=pts_in.rows();
    int raw_cols=pts_in.cols();

    //增加一行
    pts_homo_out.resize(raw_rows+1,raw_cols);
    pts_homo_out<<pts_in, Eigen::Matrix<T,1,Dynamic>::Ones(raw_cols);
    return pts_homo_out;
}
template Eigen::MatrixXd real_to_homo_coord<double>(const Eigen::MatrixXd&);
template Eigen::MatrixXf real_to_homo_coord<float>(const Eigen::MatrixXf&);

template <class T>
void real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_in,Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_homo_out){
    int raw_rows=pts_in.rows();
    int raw_cols=pts_in.cols();

    pts_homo_out.resize(raw_rows+1,raw_cols);
    pts_homo_out<<pts_in, Eigen::Matrix<T,1,Dynamic>::Ones(raw_cols);
}
template void real_to_homo_coord<double>(const Eigen::MatrixXd&, Eigen::MatrixXd&);
template void real_to_homo_coord<float>(const Eigen::MatrixXf&, Eigen::MatrixXf&);

//输入：pts_homo_in 4*8
//输出：pts_out 3*8
template <class T>
Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic> homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_homo_in){
    Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>pts_out(pts_homo_in.rows()-1,pts_homo_in.cols());
    for(int i=0;i<pts_homo_in.rows()-1;++i)
        pts_out.row(i)=pts_homo_in.row(i).array()/pts_homo_in.bottomRows(1).array();
    
    return pts_out;
}
template Eigen::MatrixXd homo_to_real_coord<double>(const Eigen::MatrixXd&);
template Eigen::MatrixXf homo_to_real_coord<float>(const Eigen::MatrixXf&);

template <class T>
void homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_homo_in, Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_out){
    pts_out.resize(pts_homo_in.rows()-1,pts_homo_in.cols());
    for(int i=0;i<pts_homo_in.rows()-1;++i)
        pts_out.row(i)=pts_homo_in.row(i).array()/pts_homo_in.bottomRows(1).array();
    
}
template void homo_to_real_coord<double>(const Eigen::MatrixXd&, Eigen::MatrixXd&);
template void homo_to_real_coord<float>(const Eigen::MatrixXf&, Eigen::MatrixXf&);

template<class T>
bool read_all_number_txt(const std::string txt_file_name, Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic>& read_number_mat){
    if(!std::ifstream(txt_file_name.c_str())){
        std::cout<<"Error!!! Cannot read txt file "<<txt_file_name<<std::endl;
        return false;
    }
    std::ifstream filetxt(txt_file_name.c_str());
    int row_counter=0;
    std::string line;
    if(read_number_mat.rows()==0)
        read_number_mat.resize(100,9); 
    
    while(getline(filetxt,line)){
        T t;
        if(!line.empty()){
            std::stringstream ss(line);
            int colu=0;
            std::string name;
            ss>>name; 
            while(ss>>t){
                read_number_mat(row_counter,colu)=t;
                colu++;
            }
            row_counter++;
            if(row_counter>=read_number_mat.rows()) //if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows()*2,read_number_mat.cols());
        }
    }
    filetxt.close();

    read_number_mat.conservativeResize(row_counter,read_number_mat.cols());     //cut into actual rows

    return true;
}
template bool read_all_number_txt(const std::string, MatrixXd&);template bool read_all_number_txt(const std::string, MatrixXi&);

bool read_local_meas_txt(const std::string& txt_file_name, std::vector<std::string>& class_names, Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic>& read_number_mat)
{
    if(!std::ifstream(txt_file_name.c_str())){
        std::cout<<"Error!!! Cannot read txt file "<<txt_file_name<<std::endl;
        return false;
    }
    std::ifstream filetxt(txt_file_name.c_str());
    int row_counter=0;
    std::string line;
    if(read_number_mat.rows()==0)
        read_number_mat.resize(10,9); 
    
    while(getline(filetxt,line)){
        double t;
        if(!line.empty()){
            std::stringstream ss(line);
            int colu=0;
            std::string name;
            ss>>name;
            class_names.push_back(name); 
            while(ss>>t){
                read_number_mat(row_counter,colu)=t;
                colu++;
            }
            row_counter++;
            if(row_counter>=read_number_mat.rows()) //if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows()*2,read_number_mat.cols());
        }
    }
    filetxt.close();

    read_number_mat.conservativeResize(row_counter,read_number_mat.cols());     //cut into actual rows

    return true;

}