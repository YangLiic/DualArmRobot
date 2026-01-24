#pragma once
/*
note: 当前wire_control和stateEstimator的功能是耦合的，后面根据情况再考虑解耦
*/
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "robot/hw_can.h"
#include "utilities/types.h"
#include "robot/cylinder_transform.h"

namespace pg
{
typedef struct _wire_sensor_data_t{
/* 安装在中心回转体、动臂、斗杆和铲斗的角度传感器  */
double jnt_q[4]; //      
double jnt_qd[3];
double jnt_acc[3];
/* 安装在关节(动臂、斗杆和铲斗)和主泵出口(左泵和右泵)压力传感器 */
double cylinder_pressure[6];  // 安装动臂、斗杆和铲斗大小腔的压力值 
double pump_pressure[2];
double euler_q[3];  // roll, pitch, yaw
double euler_qd[3]; // rollRate, pitchRate, yawrate
}wire_sensor_data_t;

typedef struct _wire_config_t{
    bool authorization;          // 占有控制权限
    bool hydraulic_switch;       // 先导控制开关
    bool auto_idle;              // 自动怠速
    int8_t motor_grade;            // 发动机档位
    bool loud_speaker;           // 喇叭
    bool pressure_button;        
    bool estop_button; 
    bool speed_mode;             // 行走高低速
}wire_config_t;

typedef struct _wire_control_cmd_t{
wire_config_t wire_config;         // 线控参数配置
double track_valve_cmd[2];         // 履带电磁阀开度
ValveCmd jnt_valve_cmd;     // 电磁阀开度
}wire_control_cmd_t;

typedef struct _wire_state_t{
Eigen::Vector4d jnt_q;     // 关节的角度
Eigen::Vector4d jnt_qd;    // 关节的速度
Eigen::Vector4d jnt_acc;    // 关节的速度
Eigen::Vector4d jnt_f;     // 液压缸的受力
Eigen::Vector4d cylinder_velo;   // 液压缸的速度
Eigen::Vector3d cabin_q;   // cabin state: roll, pitch, yaw
Eigen::VectorXd cylinder_pressure;
Eigen::Vector2d pump_pressure;
}wire_state_t;

class WireControl:public CanInterface
{
private:
    /* data */
    wire_sensor_data_t data_ = {0};
    wire_state_t state_ = {};
    std::shared_ptr<CylinderTransform> cylinder_transform_;
    void decode() override;
    /* BAIDU PC to LIUGONG PLC */
    void sendArmValve(const ValveCmd& valves);
    void sendTrackValve(const double *valves);
    void sendMachineConfig(const wire_config_t &config);
    /* update wire state based on sensor data */
    void updateSwingState(double *jnt_state, double *cabin_state);
    void updateBoomState(double *jnt_state);
    void updateStickState(double *jnt_state);
    void updateBucketState(double *jnt_state);
    /* kinematics config parameter  */
    double four_bar_len_[4];        // 铲斗四连杆尺寸
    double jnt_state_bias_[JNT_NUM];      // 关节传感器偏差
    double jnt_state_offset_[JNT_NUM];    // 动臂油缸升到最大，斗杆和铲斗油缸为最小值时，对应的关节角度
    double four_bar_q_init_[4];     // 铲斗四连杆对应的初始角度
    double jnt_cylinder_area_[JNT_PRESS_NUM];   // 动臂、斗杆和铲斗油缸大小腔的面积
    /* 程序设定 */
    double sample_rate_;            // 程序周期
public:
    /* constructor */
    WireControl(ros::NodeHandle &nh, const std::string& port_name, int baud_rate, int time_out);
    ~WireControl(){}
    
    void sendWireCmd(const wire_control_cmd_t &wire_cmd); /* BAIDU PC to LIUGONG PLC */
    void updateWireState(); /* update machine state */

    /* data accessors */
    void getWireState(wire_state_t &state);
};

} // namespace pg
