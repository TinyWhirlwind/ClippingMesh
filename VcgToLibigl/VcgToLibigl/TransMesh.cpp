#include"mymesh.h"
#include "TransMesh.h"
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/writeOBJ.h>

void TransMesh::GetLibiglMeshFromVcgData(const CMeshO& vcg_mesh, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
	V = Eigen::MatrixXd(vcg_mesh.vn, 3);
	F = Eigen::MatrixXi(vcg_mesh.fn, 3);
	for (int i = 0; i < vcg_mesh.VN(); i++) 
	{
		for (int j = 0; j < 3; j++) 
		{
			V(i, j) = vcg_mesh.vert[i].P()[j];
		}
	}
	for (int i = 0; i < vcg_mesh.FN(); i++) 
	{
		for (int j = 0; j < 3; j++) 
		{
			F(i, j) = (int)vcg::tri::Index(vcg_mesh, vcg_mesh.face[i].V(j));
		}
	}
}

void TransMesh::GetVcgMeshFromLibiglData(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, CMeshO& vcg_mesh)
{
	vcg_mesh.Clear();
	for (int i = 0; i < V.rows(); i++)
	{
		auto vh = vcg::tri::Allocator<CMeshO>::AddVertices(vcg_mesh, 1);
		(*vh).P().Import(vcg::Point3d{ V(i,0),V(i,1) ,V(i,2) });
	}
	for (int i = 0; i < F.rows(); i++)
	{
		vcg::tri::Allocator<CMeshO>::AddFace(vcg_mesh, F(i, 0), F(i, 1), F(i, 2));
	}
	vcg_mesh.EnableAttribute();
}

void TransMesh::GetLibiglMeshFromOpenMeshData(MyMesh& openMesh, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
	V = Eigen::MatrixXd(openMesh.n_vertices(), 3);
	F = Eigen::MatrixXi(openMesh.n_faces(), 3);
	for (int i = 0; i < openMesh.n_vertices(); ++i)
	{
		MyMesh::VertexHandle vh = openMesh.vertex_handle(i);
		for (int j = 0; j < 3; j++)
		{
			V(i, j) = openMesh.point(vh)[j];
		}
	}

	for (int i = 0; i < openMesh.n_faces(); ++i)
	{
		MyMesh::FaceHandle fh = openMesh.face_handle(i);
		int j = 0;
		for (MyMesh::FaceVertexIter fv_it = openMesh.fv_iter(fh); fv_it.is_valid(); ++fv_it) 
		{
			F(i, j) = fv_it->idx();
			j++;
		}
	}
}

bool TransMesh::GetMeshdifferenceLibigl(const Eigen::MatrixXd& VA, const Eigen::MatrixXi& FA, const Eigen::MatrixXd& VB, const Eigen::MatrixXi& FB, Eigen::MatrixXd& VC, Eigen::MatrixXi& FC)
{
	bool is_success = igl::copyleft::cgal::mesh_boolean(VA, FA, VB, FB, igl::MESH_BOOLEAN_TYPE_MINUS, VC, FC);
	if (is_success)
	{
		return true;
	}
	std::cout << "Boolean operation failure........." << std::endl;
	return false;
}
