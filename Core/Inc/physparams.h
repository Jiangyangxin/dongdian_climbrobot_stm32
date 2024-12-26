/*
 * phys_params.h
 *
 *  Created on: Aug 7, 2014
 *Author: loywong
 */

#ifndef PHYSPARAMS_H_
#define PHYSPARAMS_H_

#define VERSION "V_1"

#define RAD2DEG(x) ((x) * (180.0f / 3.1415926f))
#define ABSF(x) ((x) < 0.0f ? -(x) : (x))
#define ABSI(x) ((x) < 0 ? -(x) : (x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

//================  system parameters
// 固定参数，一般不修改
// rad/s转换到rpm
const float toRPM = 9.55f;
// 系统控制周期
const float Ts = 0.001f;
// 电机控制周期
const float motorTs = 0.005f;
const int motorTick = 5; //(motorTs / Ts);
//
// 风机控制周期
const float fanTs = 0.02f;
const int fanTick = (fanTs / Ts);
// 风机升余弦给定
const float fanStep = 10.f;
const float PI = 3.1415926536f;
const float PI_2 = (PI / 2.f);
// 负压测量
const float barometerCoeff = 40.f / 4.0f;
const float adcResolution = 3.3f / 4095.f;
const float barometerOffset = 2.25f;

// CAN角度转换
const float canAngle = 8192.f / 360.f; //   0  ~ 8191 ->  0  ~ 360    将角度转换为圈
// CAN电流量程
const float C620Current = 16384.f;       // M3508 C620 -16384~16384 -> -20 ~ 20A
const float C610Current = 10000.f;       // M2006 C610 -10000~10000 -> -10 ~ 10A
const float C620ICoeff = 20.f / 16384.f; // M3508 C620 -16384~16384 -> -20 ~ 20A
const float C610ICoeff = 10.f / 10000.f; // M2006 C610 -10000~10000 -> -10 ~ 10A

// 根据实际情况修改参数
// 车轮半径
const float wheelR = 0.05f;
// 驱动电机减速比
const float ratio = 36.0f * 12.0f;//19.0f * 2.7551
// 驱动电机减速比
const float ratio2 = 36.f * 5.2941f;

//舵轮电流模式参数分辨率
const float motion_resolution = 0.0001f;
const short motion_params_coeff = 10000;
const float current_resolution = 0.001f;
const short current_params_coeff = 1000;

// 位置控制中最大线速度
const float maxVel = 0.5f; // 0.3m/s
// 位置控制中最大角速度
const float maxOmg = 1.f; // 1rad/s
// 舵轮控制中最大角速度
const float maxSteerOmg = PI; // 理论上最大11rad/s
// 舵轮控制中最大电流
const float maxCurrent = 9.5f; // M3508 C620 20A * 95%  = 19A //9.5
// 位置控制中最大线加速度
const float maxAcc = 1.f; // 1m/s^2
// 位置控制中最大角加速度
const float maxOmgAcc = 1.f; // 1rad/s^2
// 2006控制中最大角加速度
const float maxSteerOmgAcc = 1.f; // 1rad/s^2

// 吸附单元减速比
const float adsorptionRatio = 2e-5f; // can value (0-8191) to mm
const float adsorptionVelRatio = 8192.f / 60.f * adsorptionRatio;
const float xMaxAngle = 0.15f;       // rad
const float yMaxAngle = 0.06f;       // rad
// 吸附单元误差死区
const float diffDeadZone = 1.5f;
// 吸附单元最小位移(以传感器为准)
const float minPos = 0.01f; // mm
// 吸附单元最大位移(以传感器为准)
const float maxPos = 25.f; // mm
// 吸附单元最小位移(以电机为准)
const float minMotPos = 0.f; // mm
// 吸附单元最大位移(以电机为准)
const float maxMotPos = 40.f; // mm
// 吸附单元中最大线速度
const float adsorptionMaxVel = 3.f; // mm/s
// 吸附单元最大电流
const float adsorptionMaxCur = 0.8f * C610Current;
// 吸附单元复位电流
const float resetCurrent[3] = {1.5f, 1.5f, 1.5f};
// 位移传感器换算系数
const float posCoeff = 24.f / 4095.f; // mm
// 电机零点条件
const float posLow = 0.1f;
const float posHigh = 2.f;

// 电机位置信息
const float motor_x[3] = {-100.68f, 0.f, 100.68f}; // mm
const float motor_y[3] = {-58.87f, 78.f, -58.87f};
// idx0 - idx1, idx1 - idx2，用于计算平面表达式
const float motor_delta_x[2] = {-100.68f, -100.68f};
const float motor_delta_y[2] = {-136.87f, 136.87f};
// 位移传感器位置信息s
const float sensor_x[3] = {-105.f, 105.f, 31.6f}; // mm
const float sensor_y[3] = {35.f, 35.f, -40.76f};  // mm
// idx0 - idx1, idx1 - idx2，用于计算平面表达式
const float sensor_delta_x[2] = {-210.f, 73.4f};
const float sensor_delta_y[2] = {0.f, 75.76f};

//================ end of system

// 舵轮转向位置环
const float thPosPidP = 10.f;
const float thPosPidI = 10.f;
const float thPosPidD = 0.f;
const float thPosPidIband = PI / 10.f;
// 舵轮转向速度环
const float thVelPidP = 2000.f;
const float thVelPidI = 15000.f;
const float thVelPidD = 5000.0f;
const float thVelPidIband = 1.f;
// 舵轮转速速度环
const float vVelPidP = 2.5f;
const float vVelPidI = 25.0f;//100.f
const float vVelPidD = 3.0f;//4.f
const float vVelPidIband = 2000.0f; // 0.1m/s对应2000

const float fanP = 100.f;
const float fanI = 90.f;
const float fanIBand = 18.f;

// 吸附单元PID参数
struct PidParam
{
    float adPidP1[3];
    float adPidI1[3];
    float adPidD1[3];
    float adPidP2[3];
    float adPidI2[3];
    float adPidD2[3];
    float adPidDiffband[3];
    float adPidValband[3];

    PidParam()
    {
        // 吸附单元速度环
        adPidP1[0] = 1000.f;
        adPidP1[1] = 1000.f;
        adPidP1[2] = 1000.f;
        adPidI1[0] = 10000.f;
        adPidI1[1] = 10000.f;
        adPidI1[2] = 10000.f;
        adPidD1[0] = 2000.f;
        adPidD1[1] = 2000.f;
        adPidD1[2] = 2000.f;

        adPidDiffband[0] = 0.2f;
        adPidDiffband[1] = 0.2f;
        adPidDiffband[2] = 0.2f;

        adPidP2[0] = 500.f;
        adPidP2[1] = 500.f;
        adPidP2[2] = 500.f;
        adPidI2[0] = 1000.f;
        adPidI2[1] = 1000.f;
        adPidI2[2] = 1000.f;
        adPidD2[0] = 100.f;
        adPidD2[1] = 100.f;
        adPidD2[2] = 100.f;

        adPidValband[0] = 0.5f;
        adPidValband[1] = 0.5f;
        adPidValband[2] = 0.5f;
    };
};
#endif /* PHYS_PARAMS_H_ */