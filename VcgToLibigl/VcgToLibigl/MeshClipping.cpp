#include "MeshClipping.h"
#include "MeshReConstruction.h"
#include "CleanMesh.h"
#include "CutMesh.h"
#include "FillHole.h"
#include "Sweeping.h"
#include "TransMesh.h"
#include<vcg/complex/algorithms/refine.h>
class MeshClipping::PImpl
{
public:
	PImpl()
	{
		reConstructionMesh_.Clear();
		sweptVolume_.clear();
		upMesh_.Clear();
		moveDist_ = 5.0f;
		scaleValue_ = 1.02f;
	}
	~PImpl()
	{}

public:
	void deleteInnerPoint();
	void surfaceConstruction();
	void generateSweptVolume();
	bool applyBoolOperation();
public:
	CMeshO dbMesh_;
	CMeshO olMesh_;
	CMeshO bMesh_;
	CMeshO gMesh_;

	CMeshO upMesh_;

	CMeshO reConstructionMesh_;
	MyMesh sweptVolume_;
	CMeshO clipMesh_;
	Point3f moveDir_;
	Point3f cutPlaneCenter_;
	Point3f cutPlaneDir_;

	float scaleValue_;
	float moveDist_;
};

MeshClipping::MeshClipping()
{
	impl_.reset(new PImpl());
}

MeshClipping::~MeshClipping()
{

}

void MeshClipping::SetDentureBaseMesh(const CMeshO& dbMesh)
{
	impl_->dbMesh_  = dbMesh;
	vcg::tri::Append<CMeshO, CMeshO>::Mesh(impl_->upMesh_, impl_->dbMesh_);
}

void MeshClipping::SetOverLayMesh(const CMeshO& olMesh)
{
	impl_->olMesh_ = olMesh;
	vcg::tri::Append<CMeshO, CMeshO>::Mesh(impl_->upMesh_, impl_->olMesh_);
}

void MeshClipping::SetBaseMesh(const CMeshO& bMesh)
{
	impl_->bMesh_ = bMesh;
	vcg::tri::Append<CMeshO, CMeshO>::Mesh(impl_->upMesh_, impl_->bMesh_);
}

void MeshClipping::SetGuideMesh(const CMeshO& gMesh)
{
	impl_->gMesh_ = gMesh;
}

void MeshClipping::SetScaleValue(float scale_value)
{
	impl_->scaleValue_ = scale_value;
}

CMeshO MeshClipping::GetReConstructionMesh()
{
	return impl_->reConstructionMesh_;
}

void MeshClipping::SetBaseMeshDir(const Point3f& bDir)
{
	impl_->moveDir_ = bDir;
}

void MeshClipping::SetBaseMeshDist(float moveDist)
{
	impl_->moveDist_ = moveDist;
}

void MeshClipping::SetCutMeshDir(const Point3f& planeNor)
{
	impl_->cutPlaneDir_ = planeNor;
}

void MeshClipping::SetCutMeshCenter(const Point3f& planeCenter)
{
	impl_->cutPlaneCenter_ = planeCenter;
}

void MeshClipping::ApplyClipping()
{
	impl_->deleteInnerPoint();
	impl_->surfaceConstruction();
	impl_->generateSweptVolume();
	impl_->applyBoolOperation();
}

CMeshO MeshClipping::GetClipMesh()
{
	return impl_->clipMesh_;
}

void MeshClipping::PImpl::deleteInnerPoint()
{
	std::cout << "Start deleting interior points........." << std::endl;
	olMesh_.EnableAttribute();
	dbMesh_.EnableAttribute();
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(dbMesh_);
	for (int i = 0; i < 3; i++)
	{
		vcg::tri::Refine<CMeshO, vcg::tri::MidPoint<CMeshO> >(dbMesh_, vcg::tri::MidPoint<CMeshO>(&dbMesh_), 0.1, false);
		vcg::tri::Refine<CMeshO, vcg::tri::MidPoint<CMeshO> >(olMesh_, vcg::tri::MidPoint<CMeshO>(&olMesh_), 0.1, false);
	}
	CMeshO interA = dbMesh_;
	CMeshO interB = olMesh_;
	interA.EnableAttribute();
	interB.EnableAttribute();
	MeshReConstruction::FilterPoints(olMesh_, interA);
	MeshReConstruction::FilterPoints(dbMesh_, interB);
	MeshReConstruction::UpdateMesh(olMesh_);
	MeshReConstruction::UpdateMesh(dbMesh_);
	vcg::tri::Append<CMeshO, CMeshO>::Mesh(olMesh_, dbMesh_);
}

void MeshClipping::PImpl::surfaceConstruction()
{
	std::cout << "Start surface reconstruction........." << std::endl;
	olMesh_.EnableAttribute();
	vcg::tri::Clean<CMeshO>::SplitNonManifoldVertex(olMesh_, 0);
	vcg::tri::Clean<CMeshO>::SplitNonManifoldVertex(olMesh_, 1);
	vcg::tri::Clean<CMeshO>::SplitManifoldComponents(olMesh_);
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(olMesh_);
	for (int i = 0; i < 5; i++)
	{
		vcg::tri::Refine<CMeshO, vcg::tri::MidPoint<CMeshO>>(olMesh_, vcg::tri::MidPoint<CMeshO>(&olMesh_), 0.07, false);
	}
	olMesh_.face.DisableVFAdjacency();
	olMesh_.vert.DisableVFAdjacency();
	olMesh_.updateBoxAndNormals();
	CMeshO sampleMesh, reConstructionMesh;
	MeshReConstruction::PoissonDiskSampling(olMesh_, sampleMesh);
	MeshReConstruction::PoissonReConstruction(sampleMesh, reConstructionMesh);
	MeshReConstruction::Remeshing(reConstructionMesh);
	MeshReConstruction::ScaleMesh(reConstructionMesh, scaleValue_);
	CleanMesh::getMaxConnectedComponent(reConstructionMesh, reConstructionMesh_);
}

void MeshClipping::PImpl::generateSweptVolume()
{
	std::cout << "Start generating swept volume........." << std::endl;
	std::pair<CMeshO, CMeshO> cutMesh;
	CutMesh::PlaneCutMesh(cutPlaneDir_, cutPlaneCenter_, reConstructionMesh_, cutMesh);
	
	MyMesh openMesh = cutMesh.first.toOpenMesh();
	FillHole fh;
	fh.SetMesh(openMesh);
	std::vector<std::vector<MyMesh::VertexHandle>> all_boundary = fh.CheckBoundarys();
	for (auto vb : all_boundary)
	{
		std::reverse(vb.begin(), vb.end());
		SweepingMesh::GenerateSweptVolume(moveDir_, moveDist_, cutMesh.first, vb);
	}

	fh.SetMesh(cutMesh.first.toOpenMesh());
	all_boundary = fh.CheckBoundarys();
	for (int i = 0; i < all_boundary.size(); i++)
	{
		fh.ApplyFillHole(i);
	}
	sweptVolume_ = fh.GetMesh();
}

bool MeshClipping::PImpl::applyBoolOperation()
{
	std::cout << "Start Model Preprocessing........." << std::endl;
	vcg::tri::Clean<CMeshO>::MergeCloseVertex(gMesh_, 0.00058f);
	vcg::tri::Clean<CMeshO>::MergeCloseVertex(upMesh_, 0.00058f);
	vcg::tri::Allocator<CMeshO>::CompactVertexVector(gMesh_);
	vcg::tri::Allocator<CMeshO>::CompactFaceVector(gMesh_);
	vcg::tri::RequireVertexCompactness(gMesh_);
	vcg::tri::RequireFaceCompactness(gMesh_);
	vcg::tri::Allocator<CMeshO>::CompactVertexVector(upMesh_);
	vcg::tri::Allocator<CMeshO>::CompactFaceVector(upMesh_);
	vcg::tri::RequireVertexCompactness(upMesh_);
	vcg::tri::RequireFaceCompactness(upMesh_);
	//vcg::tri::io::ExporterOBJ<CMeshO>::Save(gMesh_, "D:/data/ConvexHull/gMesh_.obj", 0);
	//vcg::tri::io::ExporterOBJ<CMeshO>::Save(upMesh_, "D:/data/ConvexHull/upMesh_.obj", 0);
	std::cout << "Start executing Boolean operation........." << std::endl;
	Eigen::MatrixXd VA,VB,VC,VD,Vreusult;
	Eigen::MatrixXi FA,FB,FC,FD,Freusult;
	TransMesh::GetLibiglMeshFromVcgData(gMesh_, VA, FA);
	TransMesh::GetLibiglMeshFromOpenMeshData(sweptVolume_, VC, FC);
	TransMesh::GetLibiglMeshFromVcgData(upMesh_, VB, FB);
	//OpenMesh::IO::write_mesh(sweptVolume_, "D:/data/ConvexHull/sweptVolume_.obj");
	if (TransMesh::GetMeshdifferenceLibigl(VA, FA, VB, FB, VD, FD))
	{

		if (TransMesh::GetMeshdifferenceLibigl(VD, FD, VC, FC, Vreusult, Freusult))
		{
			TransMesh::GetVcgMeshFromLibiglData(Vreusult, Freusult, clipMesh_);
			return true;
		}
	}
	return false;
}