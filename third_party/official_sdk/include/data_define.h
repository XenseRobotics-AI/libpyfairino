#ifndef DATA_DEFINE_H
#define DATA_DEFINE_H

#include <string.h>
#include <string>
#include <vector>
#include <memory>

//#define JOINT_NUM 16

enum commandID{
    getServoVersion=0x01,               //获取固件版本
    getServoParam=0x02,                 //获取关节全参数
    setServo2Boot=0x03,                 //设置状态机进入boot模式：
    setServoParam=0x04,                 //升级关节全参数：
    upgradeFirmware=0x05,               //升级固件
    getControllerMessage=0x06,          //获取ROS2_controller信息
    humanoridServoJ=0x07,               //关节运动, 所有关节
    stopMotion = 0x08,                  //运动停止，所有关节
    humanoridControllerServoJ=0x09,     //关节运动, 指定控制器
    stopControllerMotion = 0x10,        //运动停止，指定控制器
    getJointPosition = 0x11,            //关节反馈
    getRobotMotionDone = 0x12,          //查询机器人运动是否完成
    humanoridServoJLoad=0x13,           //关节运动servoJ开始设置
    humanoridMotion=0x14,               //关节运动servoJ
    humanoridServoJClose=0x15,          //关节运动servoJ关闭
    axisEnable=0x16,                    //关节使能
    axisDisable=0x17,                   //关节去使能
    axisZeroing=0x18,                   //关节校零
    resetError=0x19,                    //清除错误
    humanoridPackLog=0x20,              //日志导出
    humanoridRecordData=0x21,           //记录数据导出
    upgradePlugin=0x22,                 //升级插件
    frconfig=0x23,                      //设置机器人配置
};



typedef enum class _ARMErrorCode{
    Success = 0,
    InvalidInput_Nullptr = -1,          // 输入为空指针
    InvalidInput_OutOfRange = -2,       // 输入越界
    InvalidOutput_Nullptr = -3,         // 输出为空指针
    TCP_ConnectErr = -4,                // TCP连接失败
    Kine_LibLoadErr = -5,               // 动态库加载失败
    Kine_KineInitErr = -6,              // 调用正逆运动学接口前运动学库未初始化
    Kine_SingularErr = -7,              // 奇异位姿
    Kine_NumberErr = -8,                // 数值错误
    Kine_PositionDeviationErr = -9,     // 位置偏差>1 mm
    Kine_PostureDeviationErr = -10,     // rpy偏差>0.5 deg
    Traj_TimePlanErr = -11,             // 轨迹运动时间规划计算失败
    Traj_JointLimitsErr = -12,          // 关节超限
    Traj_InterpErr = -13,               // 轨迹运动插补计算失败
    Traj_ArcCalcErr = -14,              // 圆弧/圆圆心计算失败
    /* ==========夹爪========== */
    TCPError_NotSocketCreate = -15,          //创建socket失败
    TCPError_NotSocketConnect = -16,         //无法连接数据端口
    TCPError_NotKeepAlive = -17,             //开启tcp心跳检测失败
    TCPError_NotNetwork = -18,               //网络断开
    TCPError_DateError = -19,                //数据异常
    /* ======================== */
    CMD_RunErr = -20,                       // 指令执行错误
    DefaultErr = -21                    // try catch捕捉到的未明确错误
}ARMErrorCode;


#endif // DATA_DEFINE_H
