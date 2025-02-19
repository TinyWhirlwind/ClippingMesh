#ifndef MESH_RECONSTRUCTION_H
#define MESH_RECONSTRUCTION_H
#include "mymesh.h"
class MeshReConstruction
{
	struct SampleParameter
	{
		int SampleNum = 1000;
		CMeshO::ScalarType Radius = 0;
		int MontecarloRate = 20;
		bool SaveMontecarlo = false;
		bool ApproximateGeodesicDistance = false;
		bool Subsample = false;
		bool RefineFlag = false;
		bool BestSampleFlag = true;
		int BestSamplePool = 10;
		bool ExactNumFlag = false;
		float ExactNumTolerance = 0.005;
		float RadiusVariance = 1.0;

		SampleParameter() = default;
	};

public:
	MeshReConstruction(const CMeshO& mesh);
	~MeshReConstruction();

public:
	static void FilterPoints(CMeshO& mesh, CMeshO& to_mesh);
	static void Remeshing(CMeshO& mesh);
	static void UpdateMesh(CMeshO& mesh);
	static void FilterFaces(CMeshO& mesh);
	static void PoissonDiskSampling(CMeshO& mesh, CMeshO& result);
	static void PoissonReConstruction(CMeshO& mesh, CMeshO& result);
	static void ScaleMesh(CMeshO& mesh, float scale);
private:
	class PImpl;
	std::shared_ptr<PImpl> impl_;
};
#endif
