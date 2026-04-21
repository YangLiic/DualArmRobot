#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_
#include "head.h"
//前向声明 CanDevice
struct CanDevice;

using namespace Eigen;
const double eps = 1e-6;
inline double rad = M_PI / 180;
inline double deg = 180 / M_PI; //弧度转换成角度系数
inline double deducrate = 262144;
constexpr int dof = 7;

using namespace std::chrono;
using boost::property_tree::ptree;
using boost::property_tree::read_ini;

class Robot
{
private:
    const double d1 = 171e-3;
    const double d3 = 250e-3;
    const double d5 = 250e-3;
    const double d7 = 0e-3;
    // double ratio=101;
    double ratio = 4;//双编码器电机
    double res = 65536;

    Matrix<double, 1, 6> tool;
    Matrix<double, 1, 7> w;
    Matrix<double, 2, 7> limit;
    Matrix<double, 1, 7> init_qref, q2motor_direct, q2motor_offset;
    Matrix<double, 4, 4> T_falan_tcp, T_falan_tcp_Inv;
    int deviceInd, canInd;
    double rad2cnt = (ratio * res) / (2 * M_PI);
    double vel2hz = (ratio * 100) / (2 * M_PI);

    uint8_t MotorsIDlist[7];

    Matrix<double, 7, 4> MDH;
    int select = 0;

public:
    // 构造函数，用于初始化机器人参数
    Robot(const std::string &config_path ,int mod);
    // 添加公有 getter 函数，返回私有成员d1,d3,d5,d7的值
    double getD1() const;
    double getD3() const;
    double getD5() const;
    double getD7() const;
    double getratio() const ;//{ return ratio; }
    
    Matrix<double, 1, 6> getTool();
    

    int speedJ(Matrix<double, 1, 7> &dq);
    // int moveLToPos(Matrix<double, 1, 6> &posd, double vel);
    MatrixXd getJointVel(int DeviceInd, int CanInd);
    int moveJToJoint1(int DeviceInd, int CanInd,Matrix<double, 1, 7> &qd, double Tf);
    int moveLToPos_Opt(int DeviceInd, int CanInd,Matrix<double, 1, 6> &posd, double vel);
    Matrix<double, 1, 7> quadratic_cost_function(const Eigen::Matrix<double, 1, 7> &q, const Eigen::Matrix<double, 1, 6> &Vtcp);
    int moveLToMultiPos(int DeviceInd, int CanInd,MatrixXd &posd, double vel);
    Matrix<double, 1, 7> get_dq_main(const Eigen::Matrix<double, 1, 7> &q, const Eigen::Matrix<double, 1, 6> &Vtcp);
    int moveLToPos_AwayLimit(int DeviceInd, int CanInd,Matrix<double, 1, 6> &posd, double vel);

    bool check_Joint_withlimit(Matrix<double, 1, 7> qc);

    // ****************************Basic*****************
    MatrixXd q2MotorAngle(Matrix<double, 1, 7> &q);
    MatrixXd MotorAngle2q(Matrix<double, 1, 7> &MotorAngle);
    MatrixXd J_tcp_crossproduct(const Matrix<double, 1, 7> &q);
    int check_workspace_access(Matrix<double, 1, 6> &pos);

    Matrix<double, 1, 7> quadratic_cost_function_AwayLimit(const Eigen::Matrix<double, 1, 7> &q, const Eigen::Matrix<double, 1, 6> &Vtcp);

    //  ***************get********************
    MatrixXd getCsp(int DeviceInd, int CanInd);

    MatrixXd getJointPos(int DeviceInd, int CanInd);
    MatrixXd getJointPos(CanDevice &dev);//socketcan
    MatrixXd getTcpPos(int DeviceInd, int CanInd);
    MatrixXd getTcpPos(CanDevice &dev);//socketcan
    Matrix4d fk(const Matrix<double, 1, 7> &q);
    int ik(const Matrix<double, 1, 6> &pos, Matrix<double, 1, 7> &q, double q2);
    int ik_opt(Matrix<double, 1, 6> &posd, Matrix<double, 1, 7> &q_current);
    
    // /************************2026-4-15 mfs update******************************** */
    // Matrix<double,1,6> fk(const Matrix<double, 1, dof> &q); 
    // int ik_analytic(const Matrix<double, 1, 6> &pos, Matrix<double, 1, dof> &q, double q2);
    // int ik_numeric(Matrix<double, 1, 6> &posd, Matrix<double, 1, dof> &q_current);
    // int ik(Matrix<double, 1, 6> &posd, Matrix<double, 1, 7> &qc);
    // /***********************2026-4-15 mfs update********************************* */
    
    // ******************************Move**************
    int servoJ(int DeviceInd, int CanInd,Matrix<double, 1, 7> &qd);
    int servoJ(CanDevice &dev,Matrix<double, 1, 7> &qd);//socketcan
    int moveLToPosIK(int DeviceInd, int CanInd,Matrix<double, 1, 6> &posd, double vel, double q2);
    int moveLToPosIK(CanDevice &dev,Matrix<double, 1, 6> &posd, double vel, double q2);

    int moveJToJoint_Tf(int DeviceInd, int CanInd,Matrix<double, 1, 7> &qd, double Tf);
    int moveJToJoint_Tf(CanDevice &dev,Matrix<double, 1, 7> &qd, double Tf);//socketcan

    int moveLToPos(int DeviceInd, int CanInd,Matrix<double, 1, 6> &posd, double vel);
    int moveLToPos(CanDevice &dev,Matrix<double, 1, 6> &posd, double vel);//socketcan
    int moveJToJoint(int DeviceInd, int CanInd,Matrix<double, 1, 7> &qd, double vel);
    int moveJToJoint(CanDevice &dev,Matrix<double, 1, 7> &qd, double vel);//socketcan
    int moveJToPos(int DeviceInd, int CanInd,Matrix<double, 1, 6> &posd, double vel);
    int moveJToPos(CanDevice &dev,Matrix<double, 1, 6> &posd, double vel);//socketcan
};

#endif