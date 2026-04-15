#ifndef CAN_BOX_H
#define CAN_BOX_H

#include "head.h"

#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[1;32m" /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

using namespace std;

inline int CanNum = 0;

#define GET_PERSIONS 8         // 获取电机当前位置指令（1字节），转化为减速机角度公式：(返回值/65536/减速比)*360
#define GET_MOTOR_ERROR 10     // 获取电机错误指令（1字节）
#define CLEAR_ERROR 11         // 清除电机错误指令（1字节）
#define SET_MOTOR_CURRENTS 28  // 设置电机为电流模式，并设置目标电流指令（5字节）
#define SET_MOTOR_SPEED 29     // 设置电机为速度模式，并设置目标速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360
#define SET_MOTOR_POSITION 30  // 设置电机为位置模式，并设置目标位置指令（5字节），下发参数为：(减速机目标角度/360)*减速比*65536
#define SET_MOTOR_MAX_SPEED 36 // 设置电机最大正向允许速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360
#define SET_MOTOR_MIN_SPEED 37 // 设置电机最小负向允许速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360

#define SET_CURRENT 28
#define SET_SPEED 29
#define SET_POSITION 30
#define GET_CSP 65
#define SET_CURRENT_AND_GET_CSP 66
#define SET_SPEED_AND_GET_CSP 67
#define SET_POSITION_AND_GET_CSP 68
// constexpr int dof = 6;
/////////////////////////////// 1 byte message ////////////////////////////////
typedef enum
{
    SET_MOTOR_ENABLE = 1,               //  (0x01) 使能电机
    DLC1_SET_MOTOR_DISABLE = 2,         //  (0x02) 失能电机
    GET_MOTOR_RUN_MODE = 3,             //  (0x03) 获取电机运行模
    GET_REG_I_Q = 4,                    //  (0x04) 获取Iq
    GET_REG_I_Q_REF = 5,                //  (0x05) 获取Iq参考
    GET_REG_SPEED = 6,                  //  (0x06) 获取速度
    GET_REG_SPEED_REF = 7,              //  (0x07) 获取速度参考
    GET_REG_CURRENT_POSITION = 8,       //  (0x08) 获取当前位置
    GET_REG_TARGET_POSITION = 9,        //  (0x09) 获取目标位置
    GET_REG_STATUS = 10,                //  (0x0A) 获取报错状态
    SET_FAULT_ACK = 11,                 //  (0x0B) 清除错误
    GET_REG_SPEED_KP = 16,              //  (0x10) 获取速度环KP
    GET_REG_SPEED_KI = 17,              //  (0x11) 获取速度环KI
    GET_REG_POSITION_KP = 18,           //  (0x12) 获取位置环KP
    GET_REG_POSITION_KD = 19,           //  (0x13) 获取位置环KD
    GET_REG_BUS_VOLTAGE = 20,           //  (0x14) 获取母线电压
    GET_REG_MAX_APP_ACCEL = 22,         //  (0x16) 获取最大加速度
    GET_REG_MIN_APP_ACCEL = 23,         //  (0x17) 获取最小加速度
    GET_REG_MAX_APP_SPEED = 24,         //  (0x18) 获取最大速度
    GET_REG_MIN_APP_SPEED = 25,         //  (0x19) 获取最小速度
    GET_REG_MAX_APP_POSITION = 26,      //  (0x1A) 获取最大位置
    GET_REG_MIN_APP_POSITION = 27,      //  (0x1B) 获取最小位置
    GET_REG_MOTOR_TEMP = 49,            //  (0x31) 获取电机温度
    GET_REG_BOARD_TEMP = 50,            //  (0x32) 获取驱动板温度
    GET_REG_CSP = 65,                   //  (0x41) 获取CSP
    SET_BREAKE_OPEN = 72,               //  (0x48) 失能刹车(开闸)
    SET_BREAKE_LOCK = 73,               //  (0x49) 使能刹车(抱闸)
    GET_ENCODER_MODE = 112,             //  (0x70) 获取编码器当前工作模式 0多圈模式(默认) 1单圈模式(只能小于±180°)
    GET_8BYTE_MODE = 113,               //  (0x71) 获取8字节指令控制模式 0力位混合 1轮廓位置模式
    GET_BAT_VOL = 120,                  //  (0x78) 获取电池电压
    GET_REG_CURRENT_POSITION_OUT = 121, //  (0x79) 获取外圈编码器位置
    GET_REG_CURRENT_KP = 133,           //  (0x85) 获取电流KP
    GET_REG_CURRENT_KI = 134,           //  (0x86) 获取电流KI
    GET_REG_MOTOR_MAX_TEMP = 143,       //  (0x8F) 获取电机最大温度
    GET_REG_BOARD_MAX_TEMP = 147,       //  (0x93) 获取驱动板最大温度
    SEND_UPGRADEFIRMWARE_INFOR = 211,   //  (0xD3)
} CAN_MSG_1BYTE_t;

/////////////////////////////// 5 byte message ////////////////////////////////
typedef enum
{
    DLC5_SET_MOTOR_DISABLE = 2,              //  (0x02) 失能电机
    SET_REG_TORQUE_REF = 28,                 // (0x1C) 设置电流参考
    SET_REG_RAMP_FINAL_SPEED = 29,           // (0x1D) 设置速度参考
    SET_REG_TARGET_POSITION = 30,            // (0x1E) 设置目标位置
    SET_REG_TARGET_POSITION_TRAPEZIUM = 31,  // (0x1F) 设置位置梯形
    SET_REG_MAX_APP_TORQUE = 32,             // (0x20) 设置最大电流
    SET_REG_MIN_APP_TORQUE = 33,             // (0x21) 设置最小电流
    SET_REG_MAX_APP_ACCEL = 34,              // (0x22) 设置加速度
    SET_REG_MIN_APP_ACCEL = 35,              // (0x23) 设置减速度
    SET_REG_MAX_APP_SPEED = 36,              // (0x24) 设置最大速度
    SET_REG_MIN_APP_SPEED = 37,              // (0x25) 设置最小速度
    SET_REG_MAX_APP_POSITION = 38,           // (0x26) 设置最大位置
    SET_REG_MIN_APP_POSITION = 39,           // (0x27) 设置最小位置
    SET_REG_SPEED_KP = 41,                   // (0x29) 设置速度环KP
    SET_REG_SPEED_KI = 42,                   // (0x2A) 设置速度环KI
    SET_REG_POSITION_KP = 43,                // (0x2B) 设置位置环KP
    SET_REG_POSITION_KI = 44,                // (0x2C) 设置位置环KI
    SET_REG_POSITION_KD = 45,                // (0x2D) 设置位置环KD
    SET_REG_CAN_ID = 46,                     // (0x2E) 设置CANID
    SET_POSITION_LIMIT_ENABLE = 47,          // (0x2F) 设置位置限制
    SET_ENCODER_POS = 58,                    // (0x3A) 设置电机电角度
    SET_PROFILE_ACC = 59,                    // (0x3B) 设置轮廓加速度
    SET_PROFILE_JERK = 60,                   // (0x3C) 设置轮廓加加速度
    SET_REG_CAN_BAUD_RATE = 63,              // (0x3F) 获取波特率
    SET_TORQUE_GET_CSP = 66,                 // (0x42) 设置电流并获取CSP
    SET_SPEED_GET_CSP = 67,                  // (0x43) 设置速度并获取CSP
    SET_POSITION_GET_CSP = 68,               // (0x44) 设置位置并获取CSP
    SET_TRAPEZIUM_POSITION_GET_CSP = 70,     // (0x46) 设置位置梯形并获取CSP
    SET_POSTION_FEEDFORWARD = 71,            // (0x47) 位置环前馈
    SET_ENCODER_ZERO = 80,                   // (0x50) 设置编码器为0
    SET_MOTOR_POS_ZERO = 83,                 // (0x53) 位置偏移量为0 编码器清零
    SET_Connection_ms = 89,                  // (0x59) 上传CSP报文时间 通信保护时间
    SET_ENCODER_MODE = 112,                  // (0x70) 设置编码器工作模式 0多圈模式(默认) 1单圈模式(只能小于±180°)
    SET_8BYTE_MODE = 113,                    // (0x71) 设置8字节指令控制模式 0力位混合 1轮廓位置模式
    SET_REG_CURRENT_KP = 131,                // (0x83) 设置电流KP
    SET_REG_CURRENT_KI = 132,                // (0x84) 设置电流KI
    SET_REG_MAX_VOLTAGE = 135,               // (0x87) 设置最大电压
    SET_REG_NORMAL_VOLTAGE = 136,            // (0x88) 设置额定电压
    SET_REG_MIN_VOLTAGE = 137,               // (0x89) 设置最小电压
    SET_REG_MOTOR_MAX_TEMP = 141,            // (0x8D) 设置电机最大温度
    SET_REG_BOARD_MAX_TEMP = 145,            // (0x91) 设置驱动板最大温度
    SET_BOOTLOADER_TIME = 213,               // (0xD5)
    CALIBRATION_COMPLETIONINFORMATION = 218, // (0xDA) 编码器校准值传输完毕
    START_CALIBRATION_SPEEDDATA = 212,       // (0xD4) 开始发送速度获得位置数据
    SEND_LEN_SPEEDDATA = 214,                // (0xD6) 开始发送速度获得位置数据
} CAN_MSG_5BYTE_t;

/////////////////////////////// 6 byte message ////////////////////////////////
typedef enum
{
    DLC_6_NULL = 0x01,

    DLC_6_MOTOR_RATIO = 0x41,
    DLC_6_POS_TORQUE_KP = 0x42,
    DLC_6_POS_TORQUE_KD = 0x43,
    DLC_6_POS_TORQUE_KT = 0x44,
    DLC_6_POS_TORQUE_KV = 0x45,

    DLC_6_POS_TORQUE_T_MINX = 0x46,
    DLC_6_POS_TORQUE_T_MAXX = 0x47,
    DLC_6_POS_TORQUE_I_MINX = 0x48,
    DLC_6_POS_TORQUE_I_MAXX = 0x49,

    DLC_6_SET_GET_NTC_TRPE = 150, // (0x96) RW 电机线圈温度传感器类型
    DLC_6_ParaCofig = 0x97,       // 位置环PID功能
    DLC_6_ParaMin = 0x98,         // 位置环PID功能最小值
    DLC_6_ParaMax = 0x99,         // 位置环PID功能最大值
    DLC_6_EcatAddr = 0x9A,        // EtherCAT 从站ID

} CAN_MSG_6BYTE_t;

/////////////////////////////// 7 byte message ////////////////////////////////
typedef enum
{
    DLC_7_POS_TORQUE_MODE = 2,  // (0x02) 七字节力位混合
    DLC7_PosInterpolation = 86, // (0x56) 位置平均插补
    SET_PP_MODE = 87,           // (0x57) 轮廓位置老版本指令(梯形速度)弃用
    SET_SPEED_POS_MODE = 88,    // (0x58) 速度位置同时控制
} CAN_MSG_7BYTE_t;



// 电机 IDs
#define PI 3.14159265359f
#define KP_MINX 0.0f
#define KP_MAXX 500.0f
#define KD_MINX 0.0f
#define KD_MAXX 5.0f
#define POS_MINX -12.5f
#define POS_MAXX 12.5f
#define SPD_MINX -18.0f
#define SPD_MAXX 18.0f

// 电机类型枚举
enum ActuatorType {
    LSG_20_90_7090 = 0,
    LSG_10_414,
    LSG_17_80_6070_new,
    LSG_14_70_5060,
    LSG_20_90_7080
};

// 电机参数结构体
struct ActuatorParams {
    int actuatorType;
    int defRatio;
    float defKT;
    float tMinX;  // 最小力矩
    float tMaxX;  // 最大力矩
    float iMinX;  // 最小电流
    float iMaxX;  // 最大电流
};

// 控制数据结构体
typedef struct {
    float pos_des_;
    float vel_des_;
    float ff_;
    float kp_;
    float kd_;
} YKSMotorData;



static const ActuatorParams actuators[] = {
    {LSG_20_90_7090, 101, 0.118f, -32.0f * 2, 32.0f * 2, -22.0f, 22.0f},
    {LSG_10_414, 51, 0.175f, -94.0f * 2, 94.0f * 2, -40.0f, 40.0f},
    {LSG_17_80_6070_new, 51, 0.096f, -19.8f * 2, 19.8f * 2, -20.0f, 20.0f},
    {LSG_14_70_5060, 51, 0.089f, -6.6f * 2, 6.6f * 2, -9.0f, 9.0f},
    {LSG_20_90_7080, 51, 0.118f, -32.0f * 2, 32.0f * 2, -22.0f, 22.0f}
    };


struct CanDevice {
    std::string name;     // 接口名（如 can0, can1）
    int sock = -1;        // 套接字句柄
    int bitrate = 1000000; // 波特率
};

extern std::vector<CanDevice> canDevices;  //存储所有 CAN 设备

void signalHandler(int signum);
std::vector<std::string> query_can();
bool Start();
bool Exit();
void can_init();

inline int32_t Hex2Decimal_4Bites(const uint8_t hexArray[4]);
inline int32_t Hex2Decimal_2Bites(const uint8_t hexArray[2]);
inline void toIntArray(int number, int *res, int size);

void sendCanCommand(int DeviceInd, int CANInd, uint8_t numOfActuator, uint8_t *canIdList, uint8_t command, uint32_t *parameterList);
void sendSimpleCanCommand(int DeviceInd, int CANInd, uint8_t numOfActuator, uint8_t *canIdList, uint8_t command, int32_t *dataList);

void Clear_Error(uint8_t DeviceInd, uint8_t canID);
void Read_CSP(VCI_CAN_OBJ *send, uint8_t DeviceInd, int *dataList);

void getPosition(uint8_t DeviceInd, uint8_t canID, int &dataList);

// void getCSP(uint8_t DeviceInd, uint8_t canID, int *dataList);
// void getCSP(uint8_t DeviceInd, uint8_t canID, std::array<int, 3> &dataList);
void setCurrent(uint8_t DeviceInd, uint8_t canID, int current, int *dataList);
void setSpeed(uint8_t DeviceInd, uint8_t canID, int speed, int *dataList);
void setSpeed(uint8_t DeviceInd, uint8_t canID, std::array<int, 3> &CSP_list, int speed);
void setPosition(uint8_t DeviceInd, uint8_t canID, int position, int *dataList);

// void getCSP(int CanDeviceId, vector<int> can_list, vector<std::array<int, 3>> &CSP_list);
void getPosition(uint8_t DeviceInd, uint8_t *MotorsIDlist, int *MotorPosition);

// simple commend
void setCurrent(uint8_t DeviceInd, uint8_t canID, int current);
void setSpeed(uint8_t DeviceInd, uint8_t canID, int speed);
void setCurrentMaxLimit(uint8_t DeviceInd, uint8_t canID, int current);
void setCurrentMinLimit(uint8_t DeviceInd, uint8_t canID, int current);
void setSpeedMaxLimit(uint8_t DeviceInd, uint8_t canID, int speed);
void setSpeedMinLimit(uint8_t DeviceInd, uint8_t canID, int speed);
void setPosition(uint8_t DeviceInd, uint8_t canID, int position);

void setPosition(uint8_t DeviceInd, uint8_t *canID, int *position);
void setPosition(uint8_t DeviceInd, int dof, uint8_t *canID, int *position);

// void Force_and_Torque_Converter(struct can_frame *frame, int motor_id, float kp, float kd, float pos, float spd, float tor);
// void PT_Mode(int start_id, int numOfActuator, YKSMotorData *mot_data, int can_sock);

// void Force_and_Torque_Converter(VCI_CAN_OBJ *message, int motor_id, float kp, float kd, float pos, float spd, float tor);
// void PT_Mode(int canIndex, int motor_id, YKSMotorData mot_data);
void getCSP(int DeviceInd,int CanInd,uint8_t *canIDList, Eigen::Matrix<double, 3, 7>& dataList);
void setPositions(int DeviceInd, int CanInd, const uint8_t* canIDs, const int* positions) ;
void setPositions(CanDevice &dev, const uint8_t* canIDs, const int* positions) ;
void setSpeeds(   uint8_t DeviceInd, const uint8_t* canIDs, const int* speeds) ;
void getMotor_Pos(uint8_t DeviceInd, uint8_t *canIDList, Eigen::Matrix<double, 1, 7>& dataList) ;

//////socketcan///////////////////////////////////////////////////////////////////////////////////////////
// 检查 CAN 接口是否存在
bool interfaceExists(const std::string &ifname);
// 启用 CAN 接口
bool enableInterface(CanDevice &dev);

// 初始化 CAN 套接字
// bool init(const std::string &dev);
bool initCanDevice(CanDevice &dev);
// 关闭套接字并禁用 CAN 接口
// void closeSocket(const std::string &dev);
// 关闭单个接口
void closeCanDevice(CanDevice &CanDevice);

// 关闭所有接口
void closeAllCanDevices();

// 发送 CAN 帧
bool sendFrame(CanDevice &dev,uint32_t id, const uint8_t *data, uint8_t len);

// 接收 CAN 帧
bool recvFrame(CanDevice &dev,uint32_t &id, uint8_t *data, uint8_t &len);

// // 将 4 字节的十六进制数组转换为整数
// int32_t Hex2Decimal_4Bites(const uint8_t hexArray[4]);

// 将整数转换为字节数组
void toIntArray(int number, uint8_t *res, int size);

// 发送 CAN 命令
void sendCanCommand(CanDevice &dev,uint8_t numOfActuator, uint8_t *canIdList, uint8_t command, uint32_t *parameterList);

// 发送简单的 CAN 命令并接收响应
void sendSimpleCanCommand(CanDevice &dev,uint8_t numOfActuator, uint8_t *canIdList, uint8_t command, int32_t *dataList);

void getCSP(CanDevice &dev, uint8_t numOfActuator, uint8_t *canIdList, Eigen::Matrix<double, 3, 7> &dataList);
// #include <iostream>
// #include <cstring>
// #include <unistd.h>
// #include <cerrno>
// #include <linux/can.h>
// #include <chrono>

bool sendFrame_socketcan(CanDevice &dev, uint32_t id, const uint8_t *data, uint8_t len, int retries = 3);
//1字节指令
void sendCanCommand_socketcan(CanDevice &dev, uint8_t numOfActuator, const uint8_t *canIdList,uint8_t command, const uint32_t *parameterList);
void new_sendSimpleCanCommand_socketcan(int socket_fd,uint8_t numOfActuator,uint8_t *canIdList,uint8_t command,int32_t *dataList);//没用到

void getCSP_socketcan(int socket_fd, uint8_t numOfActuator, uint8_t *canIdList, Eigen::Matrix<double,3,7> &dataList);
void setPositions_socketcan(CanDevice &dev,const uint8_t *canIDs, const int32_t *positions);
#endif