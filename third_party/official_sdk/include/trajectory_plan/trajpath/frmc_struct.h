#ifndef FRMC_STRUCT
#define FRMC_STRUCT
#include "fr_traj_parameters.h"

typedef struct _VECT {		/* Basic spatial 3-vector  */
    double x;
    double y;
    double z;
} VECT;


typedef struct _TRSF {		/*   4X4 homogenous transform:	*/
    VECT n;					/*   unit vector for new x axis */
    VECT o;					/*   unit vector for new y axis */
    VECT a;					/*   unit vector for new z axis */
    VECT p;					/*   translation vector			*/
} TRSF;



typedef struct _JNTS
{
    double v[NUM_JNTS];		/* joint values */
    char type[NUM_JNTS+1];	/* joint type ('r' or 'p') */
    int num;				/* number of actual joints */
} JNTS;



typedef struct _QMC_LIMITS_JNTS {
      /** @brief Maximum jerk value (m/s^3) */
      //double maxJerk;
      /** @brief Maximum acceleration value (degree/s^2) 20210909  */
        JNTS maxAcc;
      /** @brief Maximum velocity value (degree/s) 20210909 */
        JNTS maxVel;

} QMC_LIMITS_JNTS;



typedef struct _QMC_ACC_JNTS {
    JNTS aPs;                    //单位为degree/s^2
    JNTS aCrs;                   //单位为degree/s^2
    JNTS aNgt;                   //单位为degree/s^2
} QMC_ACC_JNTS;



typedef struct _QMC_LIMITS {//this is set speed
      /** @brief Maximum jerk value (m/s^3) */
      //double maxJerk;
      /** @brief Maximum acceleration value (mm/s^2) */
      double maxAcc;
      /** @brief Maximum velocity value (mm/s) */
      double maxVel;

      /** @brief Maximum acceleration value (degree/s^2) 20210909*/
      double maxAccRot;
      /** @brief Maximum velocity value (degree/s) 20210909 */
      double maxVelRot;

      VECT maxAccXyz;
      VECT maxVelXyz;
} QMC_LIMITS_CART;



typedef struct _QMC_TIMES {
    double tPs;//acc time
    double tCrs;//time at maxspeed
    double tNgt;//decl time
} QMC_TIMES;



typedef struct _QMC_ACC {
    //运动为直线平移运动时的加速度
    VECT aPs;
    VECT aCrs;
    VECT aNgt;

    //运动为笛卡尔圆弧运动时的角加速度
    TRSF mToBase;                //二维平面坐标转换为ref坐标系坐标
    double mRadius;
    double marcTheta;            //圆弧圆心角， 内部使用，单位为arc
    //平移圆弧运动的角加速度分段值
    double aAglCirPs;            //内部使用，单位为arc/s^2
    double aAglCirCrs;           //内部使用，单位为arc/s^2
    double aAglCirNgt;           //内部使用，单位为arc/s^2

    //姿态轨迹规划时的旋转轴及角加速度
    double aAglPs;               //内部使用，单位为arc/s^2
    double aAglCrs;              //内部使用，单位为arc/s^2
    double aAglNgt;              //内部使用，单位为arc/s^2

    //如果有spiral和helix，则需要增加响应的加速度
    double aSpPs;
    double aSpCrs;
    double aSpNgt;

    double aHlxPs;
    double aHlxCrs;
    double aHlxNgt;

    VECT aTiltPs;
    VECT aTiltCrs;
    VECT aTiltNgt;

} QMC_ACC_CART;


typedef struct _QMC_ARC_OPTION {
    /** @brief arc, full circle or multi-circle(spiral and/or helix) */
    double nbCircle;     // ==0.0, teach arc;  >0.0, part or multi-circle.

    int fTilt;           //==1,4点法确定姿态过渡；==0， 三点法作姿态调整
    TRSF tiltPoint;      //终点姿态计算辅助，结合计算sp点计算倾角
    double rx;
    double ry;
    double rz;           //姿态角度修正系数，单位为Degree

    /** @brief make a entire circle, add an additional point to set the ideal pos along the circle */
    TRSF viaPoint3;      //暂不使用

    /** @brief spiral, add-on along the radius (unit:mm) */
    double spiralAddon;

    /** @brief helix, add-on along the rotational axis (unit:mm) */
    double helixAddon;

    // The arc motion could be the combination of spiral and helix, if spiralAddon and helixAddon are not set to zero
} QMC_ARC_OPTION;

/*查文斌*/
typedef struct _LIMITSGO{
    QMC_LIMITS_CART vel;
    QMC_LIMITS_JNTS jvel;
    double q0[6];      //设置初始关节角
    QMC_TIMES tTraj;   //总时间
    QMC_ARC_OPTION arcOpt;
    QMC_ACC_CART accTraj;
    QMC_ACC_JNTS accTraj1;

} LIMITSGO;

//length: 1152
typedef struct _Matrix_POSE{
    double Tlast[4][4];         // 上一点变换矩阵
    double T0[4][4];            // 起始点变换矩阵
    double Tf[4][4];            // 终止点变换矩阵
    double Tm[4][4];            // 中间点变换矩阵-圆弧
    double Ttool[4][4];
    double Twobj[4][4];
    double Tbase[4][4];
    double Ttoollink[4][4];
    JNTS   q0;                   // 初始点关节角(deg)，用于PTP
    JNTS   qf;                   // 终止点关节角(deg)，用于PTP
} Matrix_POSE;

//记录当前环境下的参考坐标系
typedef struct _REF_COORDDINATE_SYSTEM
{
    double Ttool[4][4];     //相对与末端法兰中心点
    double Twobj[4][4];     //相对与世界坐标系
    double Tbase[4][4];     //相对与世界坐标系
    double Ttoollink[4][4]; //相对与末端法兰中心点
}REF_COORD;

typedef struct _link_POSE{
    double xyz[8][3];
    double rpy[8][3];
    double quat[8][4];
} Link_POSE;

typedef struct _Wave_Param{
    int mode;//摆动方式 1-平面正弦波摆动 2-平面三角波摆动 3-平面圆摆动 4-L型摆动
    int cir_orition;//摆动圆正逆方向，0-正，1-逆
    double F;//摆动频率，单位是HZ
    double A;//摆动幅度,单位是mm
    double left_wait;//左停留时间，单位是s
    double right_wait;//右停留时间，单位是s
} Wave_Param;


typedef struct _Motion_control_Param{
    Wave_Param wave_param;
} Motion_control_Param;

#endif // FRMC_STRUCT

