#include "MeshClipping.h"
#include <wrap/io_trimesh/import_stl.h>
#include <wrap/io_trimesh/export_stl.h>
#include <wrap/io_trimesh/export_obj.h>
#include <wrap/io_trimesh/import_obj.h>
#include <wrap/io_trimesh/export_ply.h>
#include <igl/writeSTL.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include<wrap/io_trimesh/io_ply.h>
#include<vcg/complex/algorithms/refine.h>
#include <vcg/complex/algorithms/isotropic_remeshing.h>
#include <Eigen/core>
#include <QtCore/QDebug>
#include <QtCore/QMessageLogger>
#include <QtNetwork/QSslSocket>
int main()
{

	CMeshO inputMesh;
	CMeshO inputMeshKit;
	CMeshO inputMeshBottom;
	CMeshO inputMeshDown;
	CMeshO outputMesh;
	int mark = 0;
	std::string input_name = "D:\\data\\ConvexHull\\11\\upCover.STL";
	std::string input_name_kit = "D:\\data\\ConvexHull\\11\\upKit.STL";
	std::string input_name_bottom = "D:\\data\\ConvexHull\\11\\bottom.STL";
	std::string input_name_down = "D:\\data\\ConvexHull\\11\\down.STL";
	std::string output_name = "D:/data/ConvexHull/ClipMesh.obj";
	vcg::tri::io::ImporterSTL<CMeshO>::OpenBinary(inputMesh, input_name.c_str(), mark);
	vcg::tri::io::ImporterSTL<CMeshO>::OpenBinary(inputMeshKit, input_name_kit.c_str(), mark);
	vcg::tri::io::ImporterSTL<CMeshO>::OpenBinary(inputMeshBottom, input_name_bottom.c_str(), mark);
	vcg::tri::io::ImporterSTL<CMeshO>::OpenBinary(inputMeshDown, input_name_down.c_str(), mark);
	MeshClipping mc;
	mc.SetDentureBaseMesh(inputMeshKit);//基托
	mc.SetOverLayMesh(inputMesh);//盖片
	mc.SetBaseMesh(inputMeshBottom);//网底
	mc.SetGuideMesh(inputMeshDown);//导板
	mc.SetScaleValue(1.03f);//导板
	CMeshO newUpMesh = mc.GetReConstructionMesh();
	mc.SetBaseMeshDir(Point3f{ -1,0,0 }.Normalize());//移动方向
	mc.SetBaseMeshDist(5.0f);//移动距离
	mc.SetCutMeshDir(Point3f{ 1,1.5,0 });
	mc.SetCutMeshCenter(newUpMesh.bbox.Center() + Point3f{ 2.5,0,0 });
	mc.ApplyClipping();
	outputMesh = mc.GetClipMesh();
	vcg::tri::io::ExporterOBJ<CMeshO>::Save(outputMesh, output_name.c_str(), 0);
	/*
	inputMesh.EnableAttribute();
	inputMeshKit.EnableAttribute();
	//MeshReConstruction::EnableAttribute(inputMeshBottom);
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(inputMeshKit);
	for (int i = 0; i < 3; i++)
	{
		vcg::tri::Refine<CMeshO, vcg::tri::MidPoint<CMeshO> >(inputMeshKit, vcg::tri::MidPoint<CMeshO>(&inputMeshKit), 0.1, false);
		vcg::tri::Refine<CMeshO, vcg::tri::MidPoint<CMeshO> >(inputMesh, vcg::tri::MidPoint<CMeshO>(&inputMesh), 0.1, false);
	}
	CMeshO interA = inputMeshKit;
	CMeshO interB = inputMesh;
	//CMeshO interC = inputMeshBottom;
	interA.EnableAttribute();
	interB.EnableAttribute();
	//MeshReConstruction::Remeshing(interA);
	//MeshReConstruction::Remeshing(interB);
	//MeshReConstruction::Remeshing(interC);
	MeshReConstruction::FilterPoints(inputMesh, interA);
	MeshReConstruction::FilterPoints(inputMeshKit, interB);
	//MeshReConstruction::FilterPoints(inputMeshKit, interC);
	//MeshReConstruction::FilterPoints(inputMeshBottom, interA);
	//MeshReConstruction::FilterPoints(inputMeshBottom, interB);
	MeshReConstruction::UpdateMesh(inputMesh);
	MeshReConstruction::UpdateMesh(inputMeshKit);
	//MeshReConstruction::UpdateMesh(inputMeshBottom);
	vcg::tri::Append<CMeshO, CMeshO>::Mesh(inputMesh, inputMeshKit);
	//vcg::tri::Append<CMeshO, CMeshO>::Mesh(inputMesh, inputMeshBottom);
	output_name = "D:/data/ConvexHull/deleteInnerMesh.obj";
	vcg::tri::io::ExporterOBJ<CMeshO>::Save(inputMesh, output_name.c_str(), 0);
	inputMesh.EnableAttribute();
	vcg::tri::Clean<CMeshO>::SplitNonManifoldVertex(inputMesh, 0);
	vcg::tri::Clean<CMeshO>::SplitNonManifoldVertex(inputMesh, 1);
	vcg::tri::Clean<CMeshO>::SplitManifoldComponents(inputMesh);
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(inputMesh);
	for (int i = 0; i < 5; i++)
	{
		vcg::tri::Refine<CMeshO, vcg::tri::MidPoint<CMeshO>>(inputMesh, vcg::tri::MidPoint<CMeshO>(&inputMesh), 0.07, false);
	}
	inputMesh.face.DisableVFAdjacency();
	inputMesh.vert.DisableVFAdjacency();
	inputMesh.updateBoxAndNormals();
	CMeshO sampleMesh, reConstructionMesh;
	MeshReConstruction::PoissonDiskSampling(inputMesh, sampleMesh);
	output_name = "D:/data/ConvexHull/sampleMesh.obj";
	vcg::tri::io::ExporterOBJ<CMeshO>::Save(sampleMesh, output_name.c_str(), 0);
	MeshReConstruction::PoissonReConstruction(sampleMesh, reConstructionMesh);
	MeshReConstruction::Remeshing(reConstructionMesh);
	output_name = "D:/data/ConvexHull/reConstructionMesh.obj";
	vcg::tri::io::ExporterOBJ<CMeshO>::Save(reConstructionMesh, output_name.c_str(), 0);
	MeshReConstruction::ScaleMesh(reConstructionMesh);
	CMeshO resultMesh;
	CleanMesh::getMaxConnectedComponent(reConstructionMesh, resultMesh);

	std::pair<CMeshO, CMeshO> cutMesh;
	CutMesh::PlaneCutMesh(Point3f{ 1,1.5,0 }, resultMesh.bbox.Center() + Point3f{ 2.5,0,0 }, resultMesh, cutMesh);
    output_name = "D:/data/ConvexHull/cut_left.obj";
	vcg::tri::io::ExporterOBJ<CMeshO>::Save(cutMesh.first, output_name.c_str(), 0);
	output_name = "D:/data/ConvexHull/cut_right.obj";
	vcg::tri::io::ExporterOBJ<CMeshO>::Save(cutMesh.second, output_name.c_str(), 0);

	MyMesh openMesh = cutMesh.first.toOpenMesh();
	FillHole fh;
	fh.SetMesh(openMesh);
	std::vector<std::vector<MyMesh::VertexHandle>> all_boundary = fh.CheckBoundarys();
	for (auto vb : all_boundary)
	{
		std::reverse(vb.begin(), vb.end());
		SweepingMesh::GenerateSweptVolume(Point3f{ -1,0,0 }.Normalize(), abs(resultMesh.bbox.DimX()) * 0.5, cutMesh.first, vb);
	}

	fh.SetMesh(cutMesh.first.toOpenMesh());
	all_boundary = fh.CheckBoundarys();
	for (int i = 0; i < all_boundary.size(); i++)
	{
		fh.ApplyFillHole(i);
	}
	std::string file_path = "D:/data/ConvexHull/SweptVolume.obj";
	OpenMesh::IO::write_mesh(fh.GetMesh(), file_path);
*/

}