/**
 * @file hw_wire_control.cpp
 * @author {qianlingfeng} (qianlingfeng01@baidu.com)
 * @brief wire control file
 * @version 0.1
 * @date 2022-11-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <cfloat>
#include "robot/hw_wire_control.h"
#include "utilities/filter.h"
#include "utilities/utilities.h"

namespace pg {

void WireControl::getWireState(wire_state_t &state) {
    state = state_;
}

WireControl::WireControl(ros::NodeHandle &nh, const std::string &port_name, int baud_rate, int time_out) :
        CanInterface(port_name, baud_rate, time_out) {
    cylinder_transform_ = std::make_shared<CylinderTransform>(nh);
    // 获取主体参数
    XmlRpc::XmlRpcValue param_list;
    if (!nh.getParam("kinematics/FourbarLen", param_list)) {
        ROS_ERROR("Failed to get parameter from server.");
    }
    for (size_t i = 0; i < param_list.size(); ++i) {
        XmlRpc::XmlRpcValue tmp_value = param_list[i];
        if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            four_bar_len_[i] = static_cast<double>(tmp_value);
        }
    }

    if (!nh.getParam("kinematics/JointStateBias", param_list)) {
        ROS_ERROR("Failed to get parameter from server.");
    }
    for (size_t i = 0; i < param_list.size(); ++i) {
        XmlRpc::XmlRpcValue tmp_value = param_list[i];
        if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            jnt_state_bias_[i] = static_cast<double>(tmp_value);
        }
    }

    if (!nh.getParam("kinematics/JointStateOffset", param_list)) {
        ROS_ERROR("Failed to get parameter from server.");
    }
    for (size_t i = 0; i < param_list.size(); ++i) {
        XmlRpc::XmlRpcValue tmp_value = param_list[i];
        if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            jnt_state_offset_[i] = static_cast<double>(tmp_value);
        }
    }

    if (!nh.getParam("kinematics/FourbarQInit", param_list)) {
        ROS_ERROR("Failed to get parameter from server.");
    }
    for (size_t i = 0; i < param_list.size(); ++i) {
        XmlRpc::XmlRpcValue tmp_value = param_list[i];
        if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            four_bar_q_init_[i] = static_cast<double>(tmp_value);
        }
    }

    if (!nh.getParam("kinematics/JointCylinderArea", param_list)) {
        ROS_ERROR("Failed to get parameter from server.");
    }
    for (size_t i = 0; i < param_list.size(); ++i) {
        XmlRpc::XmlRpcValue tmp_value = param_list[i];
        if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            jnt_cylinder_area_[i] = static_cast<double>(tmp_value);
        }
    }

    nh.param("/hardware/SampleRate", sample_rate_, 100.);
}

/****  BAIDU PC to LIUGONG PLC ******/
void WireControl::sendWireCmd(const wire_control_cmd_t &wire_cmd) {
    // the period of this loop is 10ms
    static uint8_t heart_beat = 0;
    // ROS_INFO_STREAM("stick valve is: " <<  wire_cmd.jnt_valves[2]);
    sendArmValve(wire_cmd.jnt_valve_cmd);
    sendTrackValve(wire_cmd.track_valve_cmd);
    heart_beat++;
    if (heart_beat == 10) {
        heart_beat = 0;
        sendMachineConfig(wire_cmd.wire_config);
    }
}

void WireControl::sendArmValve(const ValveCmd &valves) {
    uint8_t frame[8] = {0};
    size_t len = 8;
    int32_t boom_stick_id = 0x18AB02FA;
    uint32_t bucket_swing_id = 0x18AB04FA;

    if (valves[JNT_BOOM] > 0) { /**< 动臂指令上升*/
        /**< (valves[1]*10)%256*/
        frame[0] = (uint16_t)(valves[JNT_BOOM] * 10) & 0x00FF;
        /**< (valves[1]*10)/256*/
        frame[1] = (uint16_t)(valves[JNT_BOOM] * 10) >> 8;
    } else {
        frame[2] = (uint16_t)(-valves[JNT_BOOM] * 10) & 0x00FF;
        frame[3] = (uint16_t)(-valves[JNT_BOOM] * 10) >> 8;
    }

    if (valves[JNT_STICK] > 0) {                                /**斗杆外摆*/
        frame[6] = (uint16_t)(valves[JNT_STICK] * 10) & 0x00FF; /**< (valves[1]*10)%256*/
        frame[7] = (uint16_t)(valves[JNT_STICK] * 10) >> 8;     /**< (valves[1]*10)/256*/
    } else {
        frame[4] = (uint16_t)(-valves[JNT_STICK] * 10) & 0x00FF;
        frame[5] = (uint16_t)(-valves[JNT_STICK] * 10) >> 8;
    }
    CanInterface::can_send(boom_stick_id, frame, len);
    /**铲斗外摆*/
    if (valves[JNT_BUCKET] > 0) {
        frame[0] = 0;
        frame[1] = 0;
        frame[2] = (uint16_t)(valves[JNT_BUCKET] * 10) & 0x00FF; /**< (valves[1]*10)%256*/
        frame[3] = (uint16_t)(valves[JNT_BUCKET] * 10) >> 8;     /**< (valves[1]*10)/256*/
    } else {
        frame[0] = (uint16_t)(-valves[JNT_BUCKET] * 10) & 0x00FF; /**< 0，1 是铲斗内收阀的指令；2，3是铲斗外摆阀的指令*/
        frame[1] = (uint16_t)(-valves[JNT_BUCKET] * 10) >> 8;
        frame[2] = 0;
        frame[3] = 0;
    }

    if (valves[JNT_SWING] > 0) {                                /**左回转*/
        frame[4] = (uint16_t)(valves[JNT_SWING] * 10) & 0x00FF; /**< (valves[1]*10)%256*/
        frame[5] = (uint16_t)(valves[JNT_SWING] * 10) >> 8;     /**< (valves[1]*10)/256*/
        frame[6] = 0;
        frame[7] = 0;
    } else {
        frame[4] = 0;
        frame[5] = 0;
        frame[6] = (uint16_t)(-valves[JNT_SWING] * 10) & 0x00FF;
        frame[7] = (uint16_t)(-valves[JNT_SWING] * 10) >> 8;
    }
    CanInterface::can_send(bucket_swing_id, frame, len);
}

void WireControl::sendTrackValve(const double *valves) {
    uint8_t frame[8] = {0};
    uint32_t can_id = 0x18AB03FA;
    size_t len = 8;

    if (valves[LEFT_TRACK] > 0) {                                /**< 动臂指令上升*/
        frame[0] = (uint16_t)(valves[LEFT_TRACK] * 10) & 0x00FF; /**< (valves[1]*10)%256*/
        frame[1] = (uint16_t)(valves[LEFT_TRACK] * 10) >> 8;     /**< (valves[1]*10)/256*/
    } else {
        frame[4] = (uint16_t)(-valves[LEFT_TRACK] * 10) & 0x00FF;
        frame[5] = (uint16_t)(-valves[LEFT_TRACK] * 10) >> 8;
    }
    // 6,7 right
    if (valves[RIGHT_TRACK] > 0) {                                /**斗杆外摆*/
        frame[6] = (uint16_t)(valves[RIGHT_TRACK] * 10) & 0x00FF; /**< (valves[1]*10)%256*/
        frame[7] = (uint16_t)(valves[RIGHT_TRACK] * 10) >> 8;     /**< (valves[1]*10)/256*/
    } else {
        frame[2] = (uint16_t)(-valves[RIGHT_TRACK] * 10) & 0x00FF;
        frame[3] = (uint16_t)(-valves[RIGHT_TRACK] * 10) >> 8;
    }
    CanInterface::can_send(can_id, frame, len);
}

void WireControl::sendMachineConfig(const wire_config_t &config) {
    uint8_t frame[8] = {0};
    static uint8_t heart_beat = 0;
    uint32_t can_id = 0x18AB01FA;
    size_t len = 8;
    heart_beat++;
    frame[0] = (heart_beat % 2) ? 0x55 : 0xAA;
    frame[1] = 101;
    frame[2] = 101;
    frame[3] = (config.authorization) ? 103 : 101;
    frame[4] = (config.hydraulic_switch) ? 101 : 102;
    frame[5] = (config.speed_mode) ? 102 : 101;
    frame[6] = config.motor_grade;
    frame[7] = (uint8_t)(config.loud_speaker | (config.pressure_button << 2) 
                        | (config.auto_idle << 4) | (config.estop_button << 6)); 
    CanInterface::can_send(can_id, frame, len);
}

/****  update machine state based on wire_control data ******/
void WireControl::updateWireState() {
    static double swing_state[3];
    static double boom_state[3];
    static double stick_state[3];
    static double bucket_state[3];
    static double cabin_state[3];
    double *state_ref[JNT_NUM] = {swing_state, boom_state, stick_state, bucket_state};
    updateSwingState(swing_state, cabin_state);
    updateBoomState(boom_state);
    updateStickState(stick_state);
    updateBucketState(bucket_state);
    for (uint8_t i = JNT_SWING; i <= JNT_BUCKET; i++) {
        state_.jnt_q[i] = *(state_ref[i] + 0);  //%% 这么写比较危险，后期考虑优化
        state_.jnt_qd[i] = *(state_ref[i] + 1);
        state_.jnt_acc[i] = *(state_ref[i] + 2);
        state_.jnt_f[i] = *(state_ref[i] + 3);
    }
    state_.cylinder_pressure = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(data_.cylinder_pressure, JNT_PRESS_NUM);
    state_.pump_pressure = Eigen::Map<Eigen::Vector2d, Eigen::Unaligned>(data_.pump_pressure, PUMP_NUM);
    Eigen::Vector4d cylinder_len;
    cylinder_transform_->JntStateToCylinderState(state_.jnt_q, state_.jnt_qd, cylinder_len, state_.cylinder_velo);
    state_.cabin_q = Eigen::Map<Eigen::Vector3d>(cabin_state, 3, 1);
}

void WireControl::updateSwingState(double *jnt_state, double *cabin_state) {
    double jnt_deg = 0;
    /**通过互补滤波器计算swing的角度*/
    static MeanFilter swing_q_filter(8, 5.0);
    static MeanFilter swing_qd_filter(8, 5.0);
    double swing_q = swing_q_filter.filter(data_.jnt_q[JNT_SWING]);
    // double swing_q = data_.jnt_q[JNT_SWING];
    jnt_deg = -wrap_to_180(swing_q - jnt_state_bias_[JNT_SWING]);
    jnt_state[JNT_STATE_Q] = deg_to_radian(jnt_deg);
    double swing_qd = swing_qd_filter.filter(data_.jnt_qd[JNT_SWING]);
    jnt_state[JNT_STATE_QD] = deg_to_radian(swing_qd);
    cabin_state[ROLL] = deg_to_radian(data_.euler_q[ROLL]);
    cabin_state[PITCH] = -deg_to_radian(data_.euler_q[PITCH]);
    cabin_state[YAW] = deg_to_radian(jnt_state[JNT_STATE_Q]);  // warn: YAW后续需要更新一下
}

void WireControl::updateBoomState(double *jnt_state) {
    double jnt_deg = 0;
    double jnt_force = 0;  // unit is N
    double sample_time = 1 / sample_rate_;
    double filter_time = sample_time * 8;
    static double boom_qd_pre = 0;
    static std::shared_ptr<RCFilter> boom_acc_filter = std::make_shared<RCFilter>(filter_time, sample_time);
    static std::shared_ptr<RCFilter> boom_qd_filter = std::make_shared<RCFilter>(filter_time, sample_time);
    static std::shared_ptr<RCFilter> boom_f_filter = std::make_shared<RCFilter>(filter_time, sample_time);
    static MeanFilter boom_q_filter(8, 5.0);
    static MeanFilter boom_qd_mean_filter(8, 5.0);
    static MeanFilter pitch_q_filter(8, 5.0);
    /**直接获取角度*/
    double boom_q = boom_q_filter.filter(data_.jnt_q[JNT_BOOM]);
    double pitch_q = pitch_q_filter.filter(data_.euler_q[PITCH]);
    jnt_deg = wrap_to_180(boom_q - jnt_state_bias_[JNT_BOOM] - pitch_q); /**< 动臂下降角度值°为正*/
    /**< DH坐标系方向定义下：动臂下降为负*/
    jnt_state[JNT_STATE_Q] = jnt_state_offset_[JNT_BOOM] + deg_to_radian(jnt_deg);
    /**通过低通滤波器计算角速度*/
    double boom_qd = boom_qd_mean_filter.filter(data_.jnt_qd[JNT_BOOM]);
    jnt_state[JNT_STATE_QD] = -deg_to_radian(boom_qd_filter->filter(boom_qd));
    /**通过低通滤波器计算角加速度*/
    double boom_qd_diff = (jnt_state[JNT_STATE_QD] - boom_qd_pre) / sample_time;

    boom_qd_pre = jnt_state[JNT_STATE_QD];
    jnt_state[JNT_STATE_ACC] = boom_acc_filter->filter(boom_qd_diff);
    jnt_force = (data_.cylinder_pressure[CYLINDER_BOOM_REAR_END] * jnt_cylinder_area_[CYLINDER_BOOM_REAR_END] -
                 data_.cylinder_pressure[CYLINDER_BOOM_ROD_END] * jnt_cylinder_area_[CYLINDER_BOOM_ROD_END]) *
            1000000; /**<  如果发现压强传感器有偏差，再进行调整*/
    jnt_state[JNT_STATE_F] = boom_f_filter->filter(jnt_force);
}

void WireControl::updateStickState(double *jnt_state) {
    double jnt_deg = 0;
    double jnt_force = 0;
    double sample_time = 1 / sample_rate_;
    double filter_time = sample_time * 8;
    static std::shared_ptr<RCFilter> stick_qd_filter = std::make_shared<RCFilter>(filter_time, sample_time);
    static std::shared_ptr<RCFilter> stick_f_filter = std::make_shared<RCFilter>(filter_time, sample_time);
    static MeanFilter boom_q_filter(8, 5.0);
    static MeanFilter boom_qd_mean_filter(8, 5.0);
    static MeanFilter stick_q_filter(8, 5.0);
    static MeanFilter stick_qd_mean_filter(8, 5.0);
    /**直接获取角度*/
    double boom_q = boom_q_filter.filter(data_.jnt_q[JNT_BOOM]);
    double stick_q = stick_q_filter.filter(data_.jnt_q[JNT_STICK]);
    jnt_deg = wrap_to_180(stick_q - boom_q - jnt_state_bias_[JNT_STICK]);
    /**< DH坐标系方向定义下：动臂下降为负*/
    jnt_state[JNT_STATE_Q] = jnt_state_offset_[JNT_STICK] + deg_to_radian(jnt_deg);

    /**通过低通滤波器计算角速度*/
    double boom_qd = boom_qd_mean_filter.filter(data_.jnt_qd[JNT_BOOM]);
    double stick_qd = stick_qd_mean_filter.filter(data_.jnt_qd[JNT_STICK]);
    jnt_state[JNT_STATE_QD] = -deg_to_radian(stick_qd_filter->filter(stick_qd - boom_qd));
    jnt_force = (data_.cylinder_pressure[CYLINDER_STICK_REAR_END] * jnt_cylinder_area_[CYLINDER_STICK_REAR_END] -
                 data_.cylinder_pressure[CYLINDER_STICK_ROD_END] * jnt_cylinder_area_[CYLINDER_STICK_ROD_END]) *
            1000000; /**<  如果发现压强传感器有偏差，再进行调整*/
    jnt_state[JNT_STATE_F] = stick_f_filter->filter(jnt_force);
}

void WireControl::updateBucketState(double *jnt_state) {
    double bucket_cylinder_len = 0; /**< crank相对于零位移动的角度*/
    double rocker_move_deg = 0;     /**< rocker相对于零位移动的角度*/
    double crank_w = 0;             /**< crank对应的连杆角速度*/
    double float_w = 0;             /**< float对应的连杆角速度*/
    double rocker_w = 0;            /**< rocker对应的连杆角速度*/
    double four_bar_crank_q = 0;    /**< crank对应的角度*/
    double four_bar_rocker_q = 0;   /**< rocker对应的角度*/
    double linkage_diag_square = 0; /**< 四连杆的对角线长的平方*/
    double linkage_diag = 0;        /**< 四连杆的对角线长*/
    /**<
     * 四连杆的对角线将rocker对应的角度一分为二，q1是fix,crank,diag三角形中的角度*/
    double linkage_diag_q1 = 0;
    /**<
     * 四连杆的对角线将rocker对应的角度一分为二，q2是float,rocker,diag三角形中的角度*/
    double linkage_diag_q2 = 0;
    /**< crank的延长线：crank和rocker的延长线交于一点为float的速度瞬心*/
    double linkage_extend_crank = 0;
    /**< rocker的延长线：crank和rocker的延长线交于一点为float的速度瞬心*/
    double linkage_extend_rocker = 0;
    const double BUCKET_CYLINDER_INIT = 1.68;
    const double FOUR_BAR_CROSS = 170.63 * M_PI / 180;
    const double UT_LEN = 2.2496;
    static double bucket_q_pre = 0; /**<  diff value */
    double sample_time = 1 / sample_rate_;
    double filter_time = sample_time * 8;
    static std::shared_ptr<RCFilter> bucket_f_filter = std::make_shared<RCFilter>(filter_time, sample_time);
    static std::shared_ptr<RCFilter> bucket_qd_filter = std::make_shared<RCFilter>(filter_time, sample_time);
    static std::shared_ptr<RCFilter> bucket_q_filter = std::make_shared<RCFilter>(filter_time, sample_time);
    static MeanFilter bucket_q_mean_filter(8, 5.0);
    static MeanFilter bucket_qd_mean_filter(8, 5.0);
    static MeanFilter stick_q_filter(8, 5.0);
    static MeanFilter stick_qd_mean_filter(8, 5.0);
    /**铲斗角度计算*/
#ifdef USE_SIMULATOR
    /**直接获取角度*/
    double bucket_q = bucket_q_mean_filter.filter(data_.jnt_q[JNT_BUCKET]);
    double stick_q = stick_q_filter.filter(data_.jnt_q[JNT_STICK]);
    double jnt_deg = wrap_to_180(bucket_q - stick_q - jnt_state_bias_[JNT_BUCKET]);
    /**< DH坐标系方向定义下：动臂下降为负*/
    jnt_state[JNT_STATE_Q] = jnt_state_offset_[JNT_BUCKET] + deg_to_radian(jnt_deg);

    /**通过低通滤波器计算角速度*/
    double bucket_qd = bucket_qd_mean_filter.filter(data_.jnt_qd[JNT_BUCKET]);
    double stick_qd = stick_qd_mean_filter.filter(data_.jnt_qd[JNT_STICK]);
    jnt_state[JNT_STATE_QD] = -deg_to_radian(bucket_qd_filter->filter(bucket_qd - stick_qd));    
#else
    double bucket_sensor_q = bucket_q_mean_filter.filter(data_.jnt_q[JNT_BUCKET] - data_.jnt_q[JNT_STICK] 
                        - jnt_state_bias_[JNT_BUCKET]);
    double bucket_filter_q = deg_to_radian(bucket_q_filter->filter(bucket_sensor_q));
    four_bar_crank_q = wrap_to_pi(bucket_filter_q  + four_bar_q_init_[LINKAGE_CRANK]);
    linkage_diag_square = four_bar_len_[LINKAGE_FIX] * four_bar_len_[LINKAGE_FIX] +
            four_bar_len_[LINKAGE_CRANK] * four_bar_len_[LINKAGE_CRANK] -
            2 * four_bar_len_[LINKAGE_FIX] * four_bar_len_[LINKAGE_CRANK] * std::cos(four_bar_crank_q);
    linkage_diag = std::sqrt(linkage_diag_square);
    linkage_diag_q1 = std::acos(
            (linkage_diag * linkage_diag + four_bar_len_[LINKAGE_FIX] * four_bar_len_[LINKAGE_FIX] -
             four_bar_len_[LINKAGE_CRANK] * four_bar_len_[LINKAGE_CRANK]) /
            (2 * linkage_diag * four_bar_len_[LINKAGE_FIX]));
    linkage_diag_q2 = std::acos(
            (linkage_diag * linkage_diag + four_bar_len_[LINKAGE_ROCKER] * four_bar_len_[LINKAGE_ROCKER] -
             four_bar_len_[LINKAGE_FLOAT] * four_bar_len_[LINKAGE_FLOAT]) /
            (2 * linkage_diag * four_bar_len_[LINKAGE_ROCKER]));

    four_bar_rocker_q = linkage_diag_q1 + linkage_diag_q2;
    rocker_move_deg = four_bar_rocker_q - four_bar_q_init_[LINKAGE_FIX];
    /**< 四连杆的摆动方向和机器人方向相反*/
    jnt_state[JNT_STATE_Q] = jnt_state_offset_[JNT_BUCKET] - rocker_move_deg;
    if (jnt_state[JNT_STATE_Q] < -3.0) {
        double test = 0;
    }
    /** 铲斗角速度计算 */
    linkage_extend_crank = std::abs(
            four_bar_len_[LINKAGE_FIX] / std::sin(M_PI - (M_PI - four_bar_crank_q) - (M_PI - four_bar_rocker_q)) *
            std::sin((M_PI - four_bar_rocker_q)));
    linkage_extend_rocker = std::abs(
            four_bar_len_[LINKAGE_FIX] / std::sin(M_PI - (M_PI - four_bar_crank_q) - (M_PI - four_bar_rocker_q)) *
            std::sin(M_PI - four_bar_crank_q));
    crank_w = deg_to_radian(bucket_qd_filter->filter(data_.jnt_qd[JNT_BUCKET] - data_.jnt_qd[JNT_STICK]));
    float_w = crank_w * four_bar_len_[LINKAGE_CRANK] / (linkage_extend_crank + four_bar_len_[LINKAGE_CRANK]);
    rocker_w = float_w * (linkage_extend_rocker + four_bar_len_[LINKAGE_ROCKER]) / four_bar_len_[LINKAGE_ROCKER];
    jnt_state[JNT_STATE_QD] = -rocker_w;
#endif
    double jnt_force =
            (data_.cylinder_pressure[CYLINDER_BUCKET_REAR_END] * jnt_cylinder_area_[CYLINDER_BUCKET_REAR_END] -
             data_.cylinder_pressure[CYLINDER_BUCKET_ROD_END] * jnt_cylinder_area_[CYLINDER_BUCKET_ROD_END]) *
            1000000; /**<  如果发现压强传感器有偏差，再进行调整*/
    jnt_state[JNT_STATE_F] = bucket_f_filter->filter(jnt_force);
}

void WireControl::decode() {
    uint32_t can_id;
    struct canfd_frame frame;
    CanInterface::get_frame(frame);    // 获取frame
    CanInterface::get_can_id(can_id);  // 获取can id
    switch (can_id) {
    /**<  动臂IMU*/
    case 0xCFF0443: {
        data_.jnt_q[JNT_BOOM] = (uint16_t)(frame.data[1] << 8 | frame.data[0]) * 0.01;  // 0-360
        data_.jnt_qd[JNT_BOOM] = (int16_t)(frame.data[3] << 8 | frame.data[2]) * 0.01;  // 有符号
        break;
    }
    /**< 斗杆IMU*/
    case 0xCFF0442: {
        data_.jnt_q[JNT_STICK] = (uint16_t)(frame.data[1] << 8 | frame.data[0]) * 0.01;  // 0-360
        data_.jnt_qd[JNT_STICK] = (int16_t)(frame.data[3] << 8 | frame.data[2]) * 0.01;  // 有符号
        break;
    }
        /**<  bucketIMU */
    case 0xCFF0441: {
        data_.jnt_q[JNT_BUCKET] = (uint16_t)(frame.data[1] << 8 | frame.data[0]) * 0.01;  // 0-360
        data_.jnt_qd[JNT_BUCKET] = (int16_t)(frame.data[3] << 8 | frame.data[2]) * 0.01;  // 有符号
        break;
    }
    case 0xCFF0544: /**<  CABIN IMU */
    {
        data_.euler_q[PITCH] = (int16_t)(frame.data[3] << 8 | frame.data[2]) * 0.01;     // 0-360
        data_.euler_q[ROLL] = (int16_t)(frame.data[1] << 8 | frame.data[0]) * 0.01;      // 有符号
        data_.jnt_qd[JNT_SWING] = (int16_t)(frame.data[5] << 8 | frame.data[4]) * 0.1;  // 有符号
        break;
    }
    case 0x185: /**< swing角度*/
    {
        const float SWING_Q_RESOLUTION = 4096.0 * 76.0 / 11.0;  /**<  回转编码器的角度分辨率*/
        const float SWING_QD_RESOLUTION = 4096.0 * 76.0 / 11.0; /**<  回转编码器的角速度分辨率*/
        uint32_t raw_data = (uint32_t)(frame.data[3] << 24 | frame.data[2] << 16 | frame.data[1] << 8 | frame.data[0]);
        data_.jnt_q[JNT_SWING] = wrap_to_180(raw_data / SWING_Q_RESOLUTION * 360.0);
        raw_data = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        // data_.jnt_qd[JNT_SWING] = wrap_to_180(raw_data / SWING_QD_RESOLUTION * 360);  // 确认一下方案
        break;
    }
    case 0x18FE25F3: /**< pump pressure*/
    {
        data_.pump_pressure[LEFT_PUMP] = frame.data[0] * 0.2;      // not fit for protocol 
        data_.pump_pressure[RIGHT_PUMP] = data_.pump_pressure[LEFT_PUMP];
        break;
    }
    case 0x18FE20FD: {
        data_.cylinder_pressure[CYLINDER_STICK_ROD_END] = frame.data[0] * 0.2;
        data_.cylinder_pressure[CYLINDER_STICK_REAR_END] = frame.data[1] * 0.2;
        data_.cylinder_pressure[CYLINDER_BUCKET_ROD_END] = frame.data[2] * 0.2;
        data_.cylinder_pressure[CYLINDER_BUCKET_REAR_END] = frame.data[3] * 0.2;
        break;
    }
    case 0x18FE21FD: {
        data_.cylinder_pressure[CYLINDER_BOOM_ROD_END] = frame.data[4] * 0.2;
        data_.cylinder_pressure[CYLINDER_BOOM_REAR_END] = frame.data[5] * 0.2;
        break;
    }
    default:
        // ROS_INFO_STREAM("receive unknown data from can bus!"  <<  can_id);
        break;
    }
}

}  // namespace pg