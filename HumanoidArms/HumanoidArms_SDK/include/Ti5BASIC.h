#ifndef _Ti5BASIC_H_
#define _Ti5BASIC_H_

#include "head.h"
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <unistd.h>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <Eigen/Dense>
#include <fcntl.h>
#include <poll.h>
#include <ncurses.h>

using namespace std;

class Robot;
extern Robot ti5robot_R;
extern Robot ti5robot_L;

#define MAX_IP_ADDR_LEN 256 // 存储IP地址的最大长度

enum ArmSide
{
  LEFT_ARM, // 左臂
  RIGHT_ARM // 右臂
};

/*查询机械臂在哪个can通道上面 2026-1-27 mfs add*/
struct ArmConfig {
    std::string name;
    std::vector<uint8_t> ids;
};

//定义识别逻辑需要的配置
extern std::vector<ArmConfig> possibleArms;
/*查询机械臂在哪个can通道上面 2026-1-27 mfs add*/

/**
 * @brief 探测指定的 CAN 接口是否存在目标 ID 组
 * @return true 如果所有指定的 ID 都有回复
 */
bool detectArmOnSocket(int socket_fd, const std::vector<uint8_t>& targetIds);

int Check_CAN_interface(CanDevice* &leftArmDev, CanDevice* &rightArmDev);

// 写入调试信息到文件
void writeDebugInfoToFile(const char *func_name, const char *info);

// 输出数组的调试信息
void printArrayDebugInfo(float arr[], int size, const char *arr_name);

/*查看机器人配置信息*/
void view_robot_config_information();

/* 获取本机IP地址*/
std::string ip_address();

/*计算正运动学*/
Matrix4d robot_fk(ArmSide side,const Matrix<double, 1, 7> &q); 

/*计算逆运动学*/
int robot_ik(ArmSide side,Matrix<double, 1, 6> &posd, Matrix<double, 1, 7> &q_current); 

/*机械臂回到零位
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    dev： CAN 通信设备
*/
void mechanical_arm_origin_socketcan(ArmSide side, CanDevice &dev);

/*获取机械臂电机错误状态(创芯科技can盒专用)
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    deviceInd：can设备号
    canInd：can通道
    dataList:接收数据的数组，错误状态
      对应位为0表示无错误，为1表示出现错误
      bit0代表软件错误，如电机运行时写入FLASH等
      bit1代表过压
      bit2代表欠压
      bit4代表启动错误
      bit5代表速度反馈错误
      bit6代表过流
      bit16代表编码器通讯错误
      bit17代表电机温度过高
      bit18代表电路板温度过高
*/
void get_mechanicalarm_status(ArmSide side, int DeviceInd, int CanInd, int32_t *dataList);

/*获取机械臂电机错误状态(socketcancan专用)
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    dev： CAN 通信设备
    dataList:接收数据的数组，错误状态
      对应位为0表示无错误，为1表示出现错误
      bit0代表软件错误，如电机运行时写入FLASH等
      bit1代表过压
      bit2代表欠压
      bit4代表启动错误
      bit5代表速度反馈错误
      bit6代表过流
      bit16代表编码器通讯错误
      bit17代表电机温度过高
      bit18代表电路板温度过高
*/
void get_mechanicalarm_status(ArmSide side, CanDevice &dev, int32_t *dataList);


/*清除电机错误(创芯科技can盒专用)
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      deviceInd：can设备号
      canInd：can通道
*/
void clear_elc_error(ArmSide side, int DeviceInd, int CanInd);

/*清除电机错误(socketcancan专用)
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      dev： CAN 通信设备
*/
void clear_elc_error(ArmSide side, CanDevice &dev);

/*机械臂刹车(创芯科技can盒专用)
参数：
  side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
  canInd：can通道
  deviceInd：can设备号
返回值：
  true：成功
  false：失败
*/
bool brake(ArmSide side, int DeviceInd, int CanInd);

/*机械臂刹车(socketcancan专用)
参数：
  side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
  dev： CAN 通信设备
返回值：
  true：成功
  false：失败
*/
bool brake(ArmSide side,CanDevice &dev);

/*获取当前关节角度（创芯科技can盒专用）
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      DeviceInd：can设备号
      CanInd：can通道
  返回值:
      7 个关节的当前角度（弧度）
*/
MatrixXd get_joint(ArmSide side,int DeviceInd, int CanInd);

/*获取当前关节角度(sockecan专用)
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      dev： CAN 通信设备
  返回值:
      7 个关节的当前角度（弧度）
*/
MatrixXd get_joint(ArmSide side,CanDevice &dev);

/*获取当前位置(创芯科技can盒专用)
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      DeviceInd：can设备号
      CanInd：can通道
  返回值:
      在笛卡尔空间中的位置欧拉角（x,y,z,Roll,Pitch,Yaw）,欧拉角单位是弧度
*/
MatrixXd get_Pos(ArmSide side,int DeviceInd, int CanInd);

/*获取当前位置(socketcan专用)
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      dev： CAN 通信设备
  返回值:
      在笛卡尔空间中的位置欧拉角（x,y,z,Roll,Pitch,Yaw）,欧拉角单位是弧度
*/
MatrixXd get_Pos(ArmSide side,CanDevice &dev);

/*关节运动到目标关节角(创芯科技can盒专用)
    参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      DeviceInd：can设备号
      CanInd：can通道
      qd：目标关节角，单位弧度
      vel：运动速度，单位m/s， 默认是0.1（0.05~0.4）
*/
int moveJ_ToJoint(ArmSide side,int DeviceInd, int CanInd,Matrix<double, 1, 7> &qd, double vel);

/*关节运动到目标关节角(socketcan专用)
    参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      dev： CAN 通信设备
      qd：目标关节角，单位弧度
      vel：运动速度，单位m/s， 默认是0.1（0.05~0.4）
*/
int moveJ_ToJoint(ArmSide side,CanDevice &dev,Matrix<double, 1, 7> &qd, double vel);

/*关节运动到笛卡尔位置(创芯科技can盒专用)  xyzrxryrz
    参数：
        side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
        DeviceInd：can设备号
        CanInd：can通道
        posd：目标笛卡尔位置，单位m,弧度
        vel：运动速度，单位m/s ，默认是0.1（0.05~0.4）
*/ 
int moveJ_ToPos(ArmSide side,int DeviceInd, int CanInd,Matrix<double, 1, 6> &posd, double vel);

/*关节运动到笛卡尔位置(socketcan专用)  xyzrxryrz
    参数：
        side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
        dev： CAN 通信设备
        posd：目标笛卡尔位置，单位m,弧度
        vel：运动速度，单位m/s ，默认是0.1（0.05~0.4）
*/ 
int moveJ_ToPos(ArmSide side,CanDevice &dev,Matrix<double, 1, 6> &posd, double vel);

/*线性运动到目标位置(创芯科技can盒专用)
    参数：
        side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
        DeviceInd：can设备号
        CanInd：can通道
        posd：目标笛卡尔位置，单位m,弧度
        vel：运动速度，单位m/s ，默认是0.1（0.05~0.4）
*/
int moveL_ToPos(ArmSide side,int DeviceInd, int CanInd,Matrix<double, 1, 6> &posd, double vel);

/*线性运动到目标位置(socketcan专用)
    参数：
        side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
        dev： CAN 通信设备
        posd：目标笛卡尔位置，单位m,弧度
        vel：运动速度，单位m/s ，默认是0.1（0.05~0.4）
*/
int moveL_ToPos(ArmSide side,CanDevice &dev,Matrix<double, 1, 6> &posd, double vel);

/*获取电机当前位置(创芯科技can盒专用)
  参数：
    deviceInd：can设备号
    canInd：can通道
    MotorsTotal：电机数量
    MotorsIDlist：电机ID列表
    MotorPosition：存储电机位置数据的数组
*/
void get_motor_position(int deviceInd, int canInd, int MotorsTotal,uint8_t *MotorsIDlist,int32_t *MotorPosition);

/*获取电机当前位置(socketcan专用)
  参数：
    dev： CAN 通信设备
    MotorsTotal：电机数量
    MotorsIDlist：电机ID列表
    MotorPosition：存储电机位置数据的数组
*/
void get_motor_position(CanDevice &dev, int MotorsTotal,uint8_t *MotorsIDlist,int32_t *MotorPosition);

/*设置电机当前位置(创芯科技can盒专用)
  参数：
    deviceInd：can设备号
    canInd：can通道
    MotorsTotal：电机数量
    MotorsIDlist：电机ID列表
    MotorPosition：存储电机位置数据的数组
*/
void set_motor_position(int deviceInd, int canInd, int MotorsTotal,uint8_t *MotorsIDlist,int32_t *MotorPosition);

/*设置电机当前位置(socketcan专用)
  参数：
    dev： CAN 通信设备
    MotorsTotal：电机数量
    MotorsIDlist：电机ID列表
    MotorPosition：存储电机位置数据的数组
*/
void set_motor_position(CanDevice &dev, int MotorsTotal,uint8_t *MotorsIDlist,int32_t *MotorPosition);

/*设置为电流模式，并设置目标电流(创芯科技can盒专用)
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    current: 目标电流
    canInd：can通道
    deviceInd：can设备号
*/
void set_current_mode(ArmSide side, int deviceInd, int canInd,uint32_t current[7]);

/*设置为电流模式，并设置目标电流(socketcan专用)
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    current: 目标电流
    dev： CAN 通信设备
*/
void set_current_mode(ArmSide side,CanDevice &dev, uint32_t current[7]);

/*拖动模式(socketcan专用)
  参数：
    L_dev：左臂 CAN 通信设备
    R_dev：右臂 CAN 通信设备
*/
int dragMod(CanDevice &L_dev,CanDevice &R_dev);

#endif
