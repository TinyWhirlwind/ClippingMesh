#include "mymesh.h"
#include "vcg/complex/allocate.h"
#include "vcg/complex/base.h"
CMeshO::CMeshO() :
	vcgTriMesh(),
	sfn(0), svn(0), pvn(0), pfn(0), Tr(Matrix44m::Identity())
{
	//enableComponentsFromOtherMesh(*this);
}

CMeshO::CMeshO(const CMeshO& oth) :
	vcgTriMesh(), sfn(oth.sfn), svn(oth.svn),
	pvn(oth.pvn), pfn(oth.pfn), Tr(oth.Tr)
{
	enableComponentsFromOtherMesh(oth);
	vcg::tri::Append<vcgTriMesh, vcgTriMesh>::MeshAppendConst(*this, oth);
	textures = oth.textures;
	normalmaps = oth.normalmaps;
	imark = oth.imark;
}

CMeshO::CMeshO(CMeshO&& oth)
	: CMeshO()
{
	swap(*this, oth);
}

CMeshO::~CMeshO()
{
	//no need to call base class destructor. It is called automatically
}

Box3m CMeshO::trBB() const
{
	Box3m bb;
	bb.Add(Tr, bbox);
	return bb;
}

CMeshO& CMeshO::operator=(CMeshO oth)
{
	swap(*this, oth);
	return *this;
}

/**
 * @brief When copying a vcg mesh, it is first necessary to enable
 * all the optional fields that are enabled on the other mesh, otherwise
 * they won't be copied on this mesh...........
 */
void CMeshO::enableComponentsFromOtherMesh(const CMeshO& oth)
{
	//vertex
	if (oth.vert.IsVFAdjacencyEnabled())
		this->vert.EnableVFAdjacency();
	if (oth.vert.IsMarkEnabled())
		this->vert.EnableMark();
	if (oth.vert.IsTexCoordEnabled())
		this->vert.EnableTexCoord();
	if (oth.vert.IsCurvatureDirEnabled())
		this->vert.EnableCurvatureDir();
	if (oth.vert.IsRadiusEnabled())
		this->vert.EnableRadius();

	//face
	if (oth.face.IsQualityEnabled())
		this->face.EnableQuality();
	if (oth.face.IsMarkEnabled())
		this->face.EnableMark();
	if (oth.face.IsColorEnabled())
		this->face.EnableColor();
	if (oth.face.IsFFAdjacencyEnabled())
		this->face.EnableFFAdjacency();
	if (oth.face.IsVFAdjacencyEnabled())
		this->face.EnableVFAdjacency();
	if (oth.face.IsCurvatureDirEnabled())
		this->face.EnableCurvatureDir();
	if (oth.face.IsWedgeTexCoordEnabled())
		this->face.EnableWedgeTexCoord();

	std::vector<std::string> perVScalarAttrs, perVPointAttrs;
	std::vector<std::string> perFScalarAttrs, perFPointAttrs;

	vcg::tri::Allocator<CMeshO>::GetAllPerVertexAttribute<float>(oth, perVScalarAttrs);
	vcg::tri::Allocator<CMeshO>::GetAllPerVertexAttribute<Point3m>(oth, perVPointAttrs);
	vcg::tri::Allocator<CMeshO>::GetAllPerFaceAttribute<float>(oth, perFScalarAttrs);
	vcg::tri::Allocator<CMeshO>::GetAllPerFaceAttribute<Point3m>(oth, perFPointAttrs);

	for (const std::string& attr : perVScalarAttrs)
		vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<float>(*this, attr);
	for (const std::string& attr : perVPointAttrs)
		vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Point3m>(*this, attr);
	for (const std::string& attr : perFScalarAttrs)
		vcg::tri::Allocator<CMeshO>::AddPerFaceAttribute<float>(*this, attr);
	for (const std::string& attr : perFPointAttrs)
		vcg::tri::Allocator<CMeshO>::AddPerFaceAttribute<Point3m>(*this, attr);
}
