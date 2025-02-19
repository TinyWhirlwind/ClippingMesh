#ifndef TRANS_MESH_H
#define TRANS_MESH_H
#include "mymesh.h"
#include <Eigen/Core>
//typedef Eigen::Matrix<double, Eigen::Dynamic, 3> EigenMatrixX3m;
class TransMesh
{
public:
    static void GetLibiglMeshFromVcgData(const CMeshO& vcg_mesh, Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    static void GetVcgMeshFromLibiglData(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, CMeshO& vcg_mesh);
    static void GetLibiglMeshFromOpenMeshData(MyMesh& openMesh, Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    static bool GetMeshdifferenceLibigl(const Eigen::MatrixXd& VA, const Eigen::MatrixXi& FA, const Eigen::MatrixXd& VB, const Eigen::MatrixXi& FB, Eigen::MatrixXd& VC, Eigen::MatrixXi& FC);
};

#endif