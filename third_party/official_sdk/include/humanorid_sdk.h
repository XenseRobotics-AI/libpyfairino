#ifndef HUMANORID_SDK_H
#define HUMANORID_SDK_H

#include "data_define.h"
#include "kinematic_lib/frmcKinematic.h"
#include <atomic>

class RobotBase
{
public:
    RobotBase();
    ~RobotBase();
    virtual ARMErrorCode MoveJ(double* destJointPos, double velocity, double acceleration, char errMsg[1024]) = 0;
    virtual ARMErrorCode StopMotion() = 0;
    virtual ARMErrorCode GetRobotMotionDone(uint8_t& state) = 0;
    virtual ARMErrorCode AxisEnable(int joint_id) = 0;
    virtual ARMErrorCode AxisDisable(int joint_id) = 0;
    virtual ARMErrorCode AxisZeroing(int joint_id) = 0;
    virtual ARMErrorCode AxisResetError() = 0;
    virtual ARMErrorCode SetLimitPositive(double* limitPositive) = 0;
    virtual ARMErrorCode SetLimitNegative(double* limitNegative) = 0;
    virtual ARMErrorCode GetJointSoftLimitDeg(double* limitNegative, double* limitPositive) = 0;
    virtual ARMErrorCode SetAccScale(double scale) = 0;
    virtual ARMErrorCode SetSpeed(double speed) = 0;

protected:
    uint8_t _controller_id;
};

class  SingleRobot : public RobotBase{
public:
    enum RobotType{
        ART3_R7 = 0,
        ART3_R7_JK_V1_0 = 1,
        ART3_R7_JK_V2_0 = 2,
        ART3_R7_BY = 3
    };
    enum RobotName{
        SingleArm = 0,
        LeftArm = 1,
        RightArm = 2,
    };
    /**
     * @brief  实例化机械臂
     * @param [in] robotype 机械臂型号
     * @param [in] robotname 单臂、左臂或者右臂
     * @param [in] 机械臂DH参数补偿值
     **/
    SingleRobot(int robotype, RobotName robotname=SingleArm, double DHCompensations[28]=nullptr);
    /**
     * @brief  关闭机械臂
     **/
    ~SingleRobot();

    /*****************控制部分******************/
    /**
     * @brief  关节空间运动
     * @param [in] destJointPos   目标点关节位置，单位deg
     * @param [in] velocity 速度百分比，范围[0-100]
     * @param [in] acceleration 加速度百分比，范围[0-100]，暂不开放
     * @param [out] errMsg 错误信息打印
     * @return 错误码
     **/
    ARMErrorCode MoveJ(double* destJointPos, double velocity, double acceleration, char errMsg[1024]);
    /**
     * @brief  笛卡尔空间点到点运动
     * @param [in] destCartPos 目标点笛卡尔位姿[mm, deg]
     * @param [in] destArmAngle 目标点机械臂臂角，单位deg
     * @param [in] velocity 速度百分比，范围[0-100]
     * @param [in] acceleration 加速度百分比，范围[0-100]，暂不开放
     * @param [out] errMsg 错误信息打印
     * @return 错误码
     **/
    ARMErrorCode MoveP(double* destCartPos, double destArmAngle, double velocity, double acceleration, char errMsg[1024]);
    /**
     * @brief  笛卡尔空间直线运动
     * @param [in] destCartPos   目标点笛卡尔位姿[mm, deg]
     * @param [in] destArmAngle 目标点机械臂臂角，单位deg
     * @param [in] velocity 速度百分比，范围[0-100]
     * @param [in] acceleration 加速度百分比，范围[0-100]，暂不开放
     * @param [out] errMsg 错误信息打印
     * @return 错误码
     **/
    ARMErrorCode MoveL(double* destCartPos, double destArmAngle, double velocity, double acceleration, char errMsg[1024]);
    /**
     * @brief  笛卡尔空间圆弧运动
     * @param [in] midCartPos   中间点笛卡尔位姿，单位 [mm deg]
     * @param [in] midArmAngle 中间点机械臂臂角，单位deg
     * @param [in] destCartPos   目标点笛卡尔位姿，单位 [mm deg]
     * @param [in] destArmAngle 目标点机械臂臂角，单位deg
     * @param [in] velocity 速度百分比，范围[0-100]
     * @param [in] acceleration 加速度百分比，范围[0-100]，暂不开放
     * @param [out] errMsg 错误信息打印
     * @return 错误码
     **/
    ARMErrorCode MoveC(double* midCartPos, double midArmAngle, double* destCartPos, double destArmAngle, double velocity, double acceleration, char errMsg[1024]);
    /**
     * @brief  笛卡尔空间伺服运动
     * @param [in] servo_cartPos   笛卡尔伺服运动轨迹点，[末端法兰坐标系位置、坐标系rpy角、机械臂臂角]，长度7*len，单位[mm、deg、deg]
     * @param [in] len   笛卡尔伺服运动轨迹点个数
     * @param [out] errMsg 错误信息打印
     * @return 错误码
     **/
    ARMErrorCode ServoCart(double* servo_cartPos, uint32_t len, char errMsg[1024]);
    /**
     * @brief 伺服运动开始，配合ServoJ指令使用
     * @return 错误码
     **/
    ARMErrorCode ServoMoveStart();
    /**
     * @brief  @brief 伺服运动结束，配合ServoJ指令使用
     * @return 错误码
     **/
    ARMErrorCode ServoMoveEnd();
    /**
     * @brief  关节空间伺服模式运动
     * @param [in] jointPos   目标关节位置，单位deg
     * @param [in] period   指令下发周期，单位ms
     * @param [out] errMsg 错误信息打印
     * @return 错误码
     **/
    ARMErrorCode ServoJ(double* jointPos, int period, char errMsg[1024]);
    /**
     * @brief  停止运动
     * @return 错误码
     **/
    ARMErrorCode StopMotion();

    /******************************************/

    /**
     * @brief  设置工具坐标系
     * @param [in] toolNum 工具坐标系id 范围1-19
     * @param [in] corrd 工具坐标系 {x, y, z, rx, ry, rz}, 单位：[mm, deg]
     * @return 错误码
     **/
    ARMErrorCode SetToolCorrd(int toolNum, double* corrd);
    /**
     * @brief  设置机械臂加速度百分比
     * @param [in] scale 加速度百分比
     * @return 错误码
     **/
    ARMErrorCode SetAccScale(double scale);
    /**
     * @brief  设置全局速度百分比
     * @param [in] speed 速度百分比
     * @return 错误码
     **/
    ARMErrorCode SetSpeed(double speed);
    /**
     * @brief  正运动学求解
     * @param [in] joint_val 关节位置，单位deg
     * @param [out] xyzrpy 笛卡尔位姿
     * @param [out] arm_angle 机械臂臂角
     * @return 错误码
     **/
    ARMErrorCode GetForwardKin(double* joint_val, double* desc_pos, double& arm_angle);
    /**
     * @brief  逆运动学求解
     * @param [in] trans 末端笛卡尔位姿，单位 [mm deg]
     * @param [in] arm_angle 机械臂臂角，单位deg
     * @param [out] joint_val 关节位置，单位deg
     * @return 错误码
     **/
    ARMErrorCode GetInverseKin(double* desc_pos, double arm_angle, double* joint_val);
    /**
     * @brief  逆运动学求解，参考指定关节位置求解
     * @param [in] trans 末端笛卡尔位姿，单位 [mm deg]
     * @param [in] arm_angle 机械臂臂角，单位deg
     * @param [in] ref_joint_val 参考关节位置，单位deg
     * @param [out] joint_val 关节位置，单位deg
     * @return 错误码
     **/
    ARMErrorCode GetInverseKinRef(double* desc_pos, double arm_angle, double* ref_joint_val, double* joint_val);
    /**
     * @brief  获取当前关节位置(角度)
     * @param [out] joint_pos，单位[deg]
     * @return 错误码
     **/
    ARMErrorCode GetActualJointPosDegree(double joint_pos[7]);
    /**
     * @brief  查询机械臂运动是否完成
     * @param [out] state，0-未完成，1-完成
     * @return 错误码
     **/
    ARMErrorCode GetRobotMotionDone(uint8_t& state);
    /**
     * @brief  获取当前末端速度
     * @param [out] eeVal，单位[mm/s、deg/s]
     * @return 错误码
     **/
    ARMErrorCode GetEEVel(double eeVal[6]);
    /**
     * @brief  关节使能
     * @param [in] joint_id 关节序号 0-机械臂所有关节，1-7-机械臂对应关节
     * @return 错误码
     **/
    ARMErrorCode AxisEnable(int joint_id);
    /**
     * @brief  关节去使能
     * @param [in] joint_id 关节序号 0-机械臂所有关节，1-7-机械臂对应关节
     * @return 错误码
     **/
    ARMErrorCode AxisDisable(int joint_id);
    /**
     * @brief  关节校零
     * @param [in] joint_id 关节序号 0-机械臂所有关节，1-7-机械臂对应关节
     * @return 错误码
     **/
    ARMErrorCode AxisZeroing(int joint_id);
    /**
     * @brief  清除控制器错误
     * @return 错误码
     **/
    ARMErrorCode AxisResetError();
    /**
     * @brief  获取当前末端法兰位姿
     * @param [in] jointPos 当前机械臂关节位置，单位deg
     * @param [out] fLangePos [末端法兰位置、姿态、机械臂臂角]，单位[mm、deg、deg]
     * @return 错误码
     **/
    ARMErrorCode GetActualToolFlangePose(double jointPos[7], double fLangePos[7]);
    /**
     * @brief  设置机械臂正限位
     * @param [in] limitPositive 关节正限位，单位deg
     * @return 错误码
     **/
    ARMErrorCode SetLimitPositive(double limitPositive[7]);
    /**
     * @brief  设置机械臂负限位
     * @param [in] limitNegative 关节负限位，单位deg
     * @return 错误码
     **/
    ARMErrorCode SetLimitNegative(double limitNegative[7]);
    /**
     * @brief  获取机械臂关节软限位角度
     * @param [out] limitNegative 关节负限位，单位deg
     * @param [out] limitPositive 关节正限位，单位deg
     * @return 错误码
     **/
    ARMErrorCode GetJointSoftLimitDeg(double limitNegative[7], double limitPositive[7]);
private:
    DHParam _dh;
    DHCMPSTPARAM _DHcmpst;
    JOINTLIMIT _jointLim;
    uint8_t _start_index;
    double _globalSpeed;  // 全局速度百分比
    double _globalAcceleration;  // 全局加速度百分比
    std::vector<double> _maxJointSpeed;
    std::vector<double> _maxJointAcceleration;
    std::vector<double> _maxCartSpeed;
    std::vector<double> _maxCartAcceleration;
    /* 判断关节位置是否超限，输入： 机械臂关节角(deg)，输出：无，返回值：是否超限 true-超限 false-不超限 */
    bool jointLimitsJudge(const std::vector<double>& joints, uint8_t& joint_id);

    RobotName _robotName;
    double _toolCorrds[20][6];
    uint8_t _toolCorrd_id;
};

class  RobotWaist : public RobotBase{
public:
    /**
     * @brief  实例化腰部
     **/
    RobotWaist();
    /**
     * @brief  关闭腰部
     **/
    ~RobotWaist();

    /**
     * @brief  关节空间运动
     * @param [in] destJointPos   目标点关节位置，单位deg
     * @param [in] velocity 速度百分比，范围[0-100]
     * @param [in] acceleration 加速度百分比，范围[0-100]，暂不开放
     * @param [out] errMsg 错误信息打印
     * @return 错误码
     **/
    ARMErrorCode MoveJ(double* destJointPos, double velocity, double acceleration, char errMsg[1024]);
    /**
     * @brief  停止运动
     * @return 错误码
     **/
    ARMErrorCode StopMotion();
    /**
     * @brief  获取当前关节位置(角度)
     * @param [out] joint_pos，单位[deg]
     * @return 错误码
     **/
    ARMErrorCode GetActualJointPosDegree(double* joint_pos);
    /**
     * @brief  查询运动是否完成
     * @param [out] state，0-未完成，1-完成
     * @return 错误码
     **/
    ARMErrorCode GetRobotMotionDone(uint8_t& state);
    /**
     * @brief  关节使能
     * @param [in] joint_id 关节序号 0-腰部所有关节使能 1,2,...-腰部对应关节
     * @return 错误码
     **/
    ARMErrorCode AxisEnable(int joint_id);
    /**
     * @brief  关节去使能
     * @param [in] joint_id 关节序号 0-腰部所有关节使能 1,2,...-腰部对应关节
     * @return 错误码
     **/
    ARMErrorCode AxisDisable(int joint_id);
    /**
     * @brief  关节校零
     * @param [in] joint_id 关节序号 0-腰部所有关节使能 1,2,...-腰部对应关节
     * @return 错误码
     **/
    ARMErrorCode AxisZeroing(int joint_id);
    /**
     * @brief  清除控制器错误
     * @return 错误码
     **/
    ARMErrorCode AxisResetError();
    /**
     * @brief  设置腰部正限位
     * @param [in] limitPositive 关节正限位，单位deg
     * @return 错误码
     **/
    ARMErrorCode SetLimitPositive(double* limitPositive);
    /**
     * @brief  设置腰部负限位
     * @param [in] limitNegative 关节负限位，单位deg
     * @return 错误码
     **/
    ARMErrorCode SetLimitNegative(double* limitNegative);
    /**
     * @brief  获取腰部关节软限位角度
     * @param [out] limitNegative 关节负限位，单位deg
     * @param [out] limitPositive 关节正限位，单位deg
     * @return 错误码
     **/
    ARMErrorCode GetJointSoftLimitDeg(double* limitNegative, double* limitPositive);
    /**
     * @brief  设置腰部加速度百分比
     * @param [in] scale 加速度百分比
     * @return 错误码
     **/
    ARMErrorCode SetAccScale(double scale);
    /**
     * @brief  设置腰部全局速度百分比
     * @param [in] speed 速度百分比
     * @return 错误码
     **/
    ARMErrorCode SetSpeed(double speed);
private:
    double _globalSpeed;  // 全局速度百分比
    double _globalAcceleration;  // 全局加速度百分比
    std::vector<double> _maxJointSpeed;
    std::vector<double> _maxJointAcceleration;
    std::vector<std::vector<double>> _jointLim;
    uint8_t _start_index;
    uint8_t _joint_num;
private:
    /* 判断关节位置是否超限，输入： 机械臂关节角(deg)，输出：无，返回值：是否超限 true-超限 false-不超限 */
    bool jointLimitsJudge(const std::vector<double>& joints, uint8_t& joint_id);
};


class RobotTool
{
public:
    /**
     * @brief  实例化工具类
     **/
    RobotTool();
    /**
     * @brief  关闭工具类对象
     **/
    ~RobotTool();

    /**
     * @brief  设置机器人配置
     * @param [in] mode 0-单臂7轴配置 1-双臂14轴配置 2-双臂+腰16轴配置
     * @return 错误码
     **/
    ARMErrorCode RobotFrConfig(int mode);
    /**
     * @brief  日志导出
     * @return 错误码
     **/
    ARMErrorCode RobotPackLog();
    /**
     * @brief  数据记录
     * @param [in] state 1-记录开始 0-记录结束并导出
     * @return 错误码
     **/
    ARMErrorCode RobotRecordData(int state);
    /**
     * @brief  ROS2插件升级
     * @param [in] filepath 升级包全路径
     * @return 错误码
     **/
    ARMErrorCode UpdateRos2Plugin(std::string filepath);
    /**
     * @brief  SDK服务端部署
     * @param [in] filepath 升级包全路径
     * @return 错误码
     **/
    ARMErrorCode UpdateSDKServer(std::string filepath);
};

#endif // HUMANORID_SDK_H
