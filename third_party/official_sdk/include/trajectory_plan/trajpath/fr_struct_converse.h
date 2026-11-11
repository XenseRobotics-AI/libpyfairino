#ifndef FR_STRUCT_CONVERSE_H
#define FR_STRUCT_CONVERSE_H

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "frmc_struct.h"
#include "fr_traj_parameters.h"


using namespace Eigen;

class FR_struct_converse
{
public:
    FR_struct_converse();
    ~FR_struct_converse();

    void TRSF2Eigen(TRSF mat, Matrix4d& eigen_mat);
    void Eigen2TRSF(Matrix4d& mat, TRSF& fr_mat);

    void Double2TRSF(double mat[4][4],TRSF& fr_mat);
    void TRSF2Double(TRSF mat, double fr_mat[4][4]);
    void Double2Eigen(double mat[4][4],Matrix4d& eigen_mat);
    void Eigen2Double(Matrix4d& eigen_mat,double mat[4][4]);

    TRSF Double2TRSF(double mat[4][4]);

    void JNTS2Eigen(JNTS jnt, VectorXd& eigen_jnt);
    void Eigen2JNTS(VectorXd eigen_jnt, JNTS& jnt);
    void JNTS2Eigen(JNTS jnt, ArrayXd& eigen_jnt);
    void Eigen2JNTS(ArrayXd eigen_jnt, JNTS& jnt);

    void VECT2Eigen(VECT vec,Vector3d& eigen_vec);
    void Eigen2VECT(Vector3d eigen_vec, VECT& vec);

    void JNTS2Double(JNTS jnt, double q[NUM_JNTS]);//注意二维数组无法使用reference
    void Double2JNTS(double q[NUM_JNTS], JNTS& jnt);

    void rpy2doublemat(double rpy[3],double mat[4][4]);
    void doublemat2rpy(double mat[4][4],double rpy[3]);

};

#endif // FR_STRUCT_CONVERSE_H
