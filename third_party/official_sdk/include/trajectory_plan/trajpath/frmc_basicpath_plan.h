#ifndef FRMC_BASICPATH_PLAN
#define FRMC_BASICPATH_PLAN
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <math.h>
#include "frmc_struct.h"
#include "fr_struct_converse.h"
#include "fr_traj_parameters.h"

extern VECT lindir_vec;
extern double lin_dis;
#ifdef __cplusplus
extern "C"
{
#endif
//PTP
int frmcComputeJointsLinMotion(JNTS startJoints, JNTS endJoints, QMC_LIMITS_JNTS limitsGoto, QMC_TIMES *tTraj, QMC_ACC_JNTS *accTraj);
int frmcComputeJointsLinAccVelPos(JNTS startJoints, JNTS endJoints, QMC_TIMES tTraj, QMC_ACC_JNTS accTraj, unsigned long numPoint, JNTS *cPoint);
int frmcComputeJointsContinueLinMotion(JNTS startJoints, JNTS endJoints, QMC_LIMITS_JNTS curlimits, QMC_LIMITS_JNTS limitsGoto, QMC_TIMES *tTraj, QMC_ACC_JNTS *accTraj);
int frmcComputeJointsContinueLinAccVelPos(JNTS startJoints, JNTS endJoints, QMC_TIMES tTraj, QMC_LIMITS_JNTS curlimits, QMC_ACC_JNTS accTraj, unsigned long numPoint, JNTS *cPoint);

//cartesean line
int frmcComputeCartLinMotion(TRSF startPoint,TRSF endPoint,QMC_LIMITS_CART limitsGoto,QMC_TIMES *tTraj, QMC_ACC_CART *accTraj);
int frmcComputeCartLinAccVelPos(TRSF startPoint, TRSF endPoint, QMC_TIMES tTraj, QMC_ACC_CART accTraj, unsigned long numPoint, TRSF *cPoint);
int frmcComputeCartContinueLinMotion(TRSF startPoint,TRSF endPoint,QMC_LIMITS_CART curlimits,QMC_LIMITS_CART limitsGoto,QMC_TIMES *tTraj, QMC_ACC_CART *accTraj);
int frmcComputeCartContinueLinAccVelPos(TRSF startPoint, TRSF endPoint, QMC_TIMES tTraj, QMC_LIMITS_CART curlimits, QMC_ACC_CART accTraj, unsigned long numPoint, TRSF *cPoint);

//cartesean arc
int frmcComputeCartArcMotion(TRSF startPoint, TRSF viaPoint, TRSF viaPoint2, QMC_ARC_OPTION arcOpt, QMC_LIMITS_CART limitsGoto, QMC_TIMES *tTraj, QMC_ACC_CART *accTraj);
int frmcComputeCartArcAccVelPos(TRSF startPoint, TRSF viaPoint, TRSF viaPoint2, QMC_ARC_OPTION arcOpt, QMC_TIMES tTraj, QMC_ACC_CART accTraj, unsigned long numPoint, TRSF *cPoint);
#ifdef __cplusplus
}
#endif
//摆动焊
int sinwave_planning(double finish_time,TRSF prevPoint, TRSF curPoint, TRSF nextPoint, Wave_Param para, int trajnum, TRSF &out_T);//平面sin波摆动
int trianglewave_planning(double finish_time,TRSF prevPoint, TRSF curPoint, TRSF nextPoint, Wave_Param para, int trajnum, TRSF &out_T);//平面三角波摆动
int circlewave_planning(double finish_time,TRSF prevPoint, TRSF curPoint, TRSF nextPoint, Wave_Param para, int trajnum, TRSF &out_T);//平面圆形波摆动
int Ltypewave_planning(double finish_time,TRSF prevPoint, TRSF curPoint, TRSF nextPoint, Wave_Param para, int trajnum, TRSF &out_T);//L型工件摆动
#endif // FRMC_BASICPATH_PLAN

