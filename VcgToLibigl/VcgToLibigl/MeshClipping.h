#ifndef MESH_CLIPPING_H
#define MESH_CLIPPING_H
#include "mymesh.h"
using namespace vcg;
class MeshClipping
{
public:
	MeshClipping();
	~MeshClipping();

public:
	/*依次调用*/
	void SetDentureBaseMesh(const CMeshO& dbMesh);//基托
	void SetOverLayMesh(const CMeshO& olMesh);//盖片
	void SetBaseMesh(const CMeshO& bMesh);//网底
	void SetGuideMesh(const CMeshO& gMesh);//导板
	void SetScaleValue(float scale_value);
	CMeshO GetReConstructionMesh();
	void SetBaseMeshDir(const Point3f& bDir);//移动方向
	void SetBaseMeshDist(float moveDist);//移动距离
	void SetCutMeshDir(const Point3f& planeNor);
	void SetCutMeshCenter(const Point3f& planeCenter);
	void ApplyClipping();
	CMeshO GetClipMesh();
private:
	class PImpl;
	std::shared_ptr<PImpl> impl_;
};
#endif