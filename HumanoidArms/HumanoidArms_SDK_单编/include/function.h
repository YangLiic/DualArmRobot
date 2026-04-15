#ifndef FUNCTION_H
#define FUNCTION_H


#include "head.h"
using namespace Eigen;
using namespace std;
// 轨迹数据结构
struct Trajectory {
    Eigen::MatrixXd coeffs;    // 多项式系数矩阵: dim × (6*n_segments)
    Eigen::MatrixXd T;         // 时间节点 (n×1 矩阵)
    int m;            // 段数
    int dim;                   // 维度数
    double tmax;               // 总时间
};

MatrixXd Transl_xyz(Matrix<double, 1, 3> trans);
MatrixXd Rot_zyx(Matrix<double, 1, 3>  theta);
MatrixXd TR(Matrix<double, 1, 6> transpose);
MatrixXd MDHTrans(double alpha, double  a, double  d, double  theta);
Matrix<double, 1, 3> rotationMatrixToEulerAngles(Matrix<double, 3, 3> R);
Matrix<double, 1, 6> T2PosEulerAngles(Matrix<double, 4, 4> T);
Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q);
MatrixXd T2Axispos(Matrix<double, 4, 4> T);
Eigen::VectorXd T2Axis(const Eigen::Matrix4d& T) ;
MatrixXd Axispos2T(Matrix<double, 1, 6> pos);
MatrixXd skew(Matrix<double, 1, 3>  s);
//正运动学 根据输入关节角度求出末端法兰相对于基座4*4矩阵
MatrixXd fkine(Matrix<double, 1, 6>  theta);
int getFileRows(const char* fileName);
int getFileColumns(const char* fileName);
double** getMatrix(const char* path, const int n, const int m);
MatrixXd line_interp(Matrix<double, 1, 6>   p1,Matrix<double, 1, 6>  p2, double dx);
MatrixXd circular_interp(Matrix<double, 1, 6>   p1,Matrix<double, 1, 6>  p2,Matrix<double, 1, 6>  p3, double dx);
MatrixXd pinv(Eigen::MatrixXd A);//计算矩阵伪逆
MatrixXd multiplyWithDOF(Matrix<double,1 , 6> dof,Matrix<double,1 , 6> forces);
MatrixXd slerp_interp(double t, Matrix<double, 1, 3> euler_start, Matrix<double, 1, 3> euler_end);

MatrixXd quinticInterp(double t, double totalTime, Matrix<double, 1, 6>& start,  Matrix<double, 1, 6>& target);
Matrix<double, 1, 6> quinticInterp_CartSpace(double t, double totalTime, const Eigen::Matrix<double, 1, 6>& start,  const Eigen::Matrix<double, 1, 6>& target);
Matrix<double, 1, 7> quinticInterp_JointSpace(double t, double totalTime, const Eigen::Matrix<double, 1, 7>& start,  const Eigen::Matrix<double, 1, 7>& target);
Matrix<double, 1, 7> quinticInterp_JointSpace_dq( double t, double totalTime,  const Eigen::Matrix<double, 1, 7>& q1,  const Eigen::Matrix<double, 1, 7>& q2) ;
double calculateTf(const Eigen::Matrix<double, 1, 6>& axis_p1, const Eigen::Matrix<double, 1, 6>& axis_p2, double V_DEFAULT) ;
double getMultiTrajTotalTime(const Eigen::MatrixXd& Q, double vel) ;
Eigen::MatrixXd multiTraj(double t, const Eigen::MatrixXd& Q, double vel);

Trajectory generateTrajVel(const Eigen::MatrixXd& P, double vel);
void multiTraj(double t, const Trajectory& traj, 
               Eigen::MatrixXd& q);
Matrix<double, 1, 7> joint_range_normalize(const Eigen::Matrix<double, 1, 7>& q,const Eigen::Matrix<double, 1, 7>& q_min,const Eigen::Matrix<double, 1, 7>& q_max);

Matrix<double, 1, 7> calc_segment_weight( const Eigen::Matrix<double, 1, 7>& x,  double w_max) ;

#endif // FUNCTION_H
