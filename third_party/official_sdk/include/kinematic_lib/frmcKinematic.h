#include "string.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/** 机器人DH参数*/
typedef struct _DHParam{
    double DH_a[7];			/** 连杆参数,单位：m*/
    double DH_d[7];			/** 连杆参数,单位：m*/
    double DH_alpha[7];		/** alpha角,单位：rad*/
    double DH_beta[7];		/** beta角,单位：rad*/
    double DH_theta[7];		/** theta角,单位：rad*/
}DHParam;

/** 机器人DH补偿参数*/
typedef struct _DHCMPSTPARAM{
    double a[7];				        /** 连杆偏距,单位：m */
    double d[7];					   	/** 连杆长度,单位：m */
    double alpha[7];					/** 连杆扭角,单位：rad */
    double beta[7];			    		/** 连杆偏角,单位：rad */
    double theta[7];					/** 关节零位,单位：rad */
}DHCMPSTPARAM;

/** 机器人关节限位参数*/
typedef struct _JOINTLIMIT{
    double joint1[2];				    /** 关节1限位,[min,max],单位：° */
    double joint2[2];				    /** 关节2限位,[min,max],单位：° */
    double joint3[2];				    /** 关节3限位,[min,max],单位：° */
    double joint4[2];				    /** 关节4限位,[min,max],单位：° */
    double joint5[2];				    /** 关节5限位,[min,max],单位：° */
    double joint6[2];				    /** 关节6限位,[min,max],单位：° */
    double joint7[2];				    /** 关节7限位,[min,max],单位：° */
}JOINTLIMIT;



#ifdef __cplusplus
extern "C" {
#endif
/*机器人正运动学解，输入：机器人dh参数(m/rad), 机器人DH补偿参数(m/rad)，关节角（°），末端关节序号(6/7)，输出：笛卡尔空间齐次矩阵（mm），七自由度机器人的臂角（°），返回值：0正确，1错误*/
int forwardKinematic(DHParam dh, DHCMPSTPARAM DHcmpst, double q[7], int jnt_index, double trans[4][4], double* armAngle);

/*机器人逆运动学解，输入：机器人dh参数(m/rad), 机器人DH补偿参数(m/rad)，机器人关节限位参数（°），末端关节序号(6/7)，七自由度机器人的臂角（°），末端齐次矩阵（mm），参考关节角（°），输出：目标关节角（°），全部关节角（6自由度机器人对应前8组，7自由度机器人对应16组），返回值：0正确，1错误*/
int inverseKinematic(DHParam dh, DHCMPSTPARAM DHcmpst,JOINTLIMIT jointLim,int jnt_index,double armAngle, double trans[4][4], double q_ref[7], double q_out[7], double q_allout[16][7]);

#ifdef __cplusplus
}
#endif
