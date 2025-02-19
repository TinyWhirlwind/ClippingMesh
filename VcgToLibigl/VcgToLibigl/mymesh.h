#ifndef MYMESH_H
#define MYMESH_H
#include <vcg/complex/complex.h>
#include "base_type.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <vcg/complex/algorithms/clean.h>
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

#define Scalarm float
namespace vcg
{
	namespace vertex
	{
		template <class T> class Coord3m : public Coord<vcg::Point3<Scalarm>, T> {
		public:	static void Name(std::vector<std::string>& name) { name.push_back(std::string("Coord3m")); T::Name(name); }
		};

		template <class T> class Normal3m : public Normal<vcg::Point3<Scalarm>, T> {
		public:	static void Name(std::vector<std::string>& name) { name.push_back(std::string("Normal3m")); T::Name(name); }
		};

		template <class T> class Qualitym : public Quality<Scalarm, T> {
		public: static void Name(std::vector<std::string>& name) { name.push_back(std::string("Qualitym")); T::Name(name); }
		};

		template <class T> class CurvatureDirmOcf : public CurvatureDirOcf<CurvatureDirTypeOcf<Scalarm>, T> {
		public:	static void Name(std::vector<std::string>& name) { name.push_back(std::string("CurvatureDirmOcf")); T::Name(name); }
		};

		template <class T> class RadiusmOcf : public RadiusOcf<Scalarm, T> {
		public:	static void Name(std::vector<std::string>& name) { name.push_back(std::string("RadiusmOcf")); T::Name(name); }
		};

	}//end namespace vertex

	namespace face
	{
		template <class T> class Normal3m : public NormalAbs<vcg::Point3<Scalarm>, T> {
		public:  static void Name(std::vector<std::string>& name) { name.push_back(std::string("Normal3m")); T::Name(name); }
		};

		template <class T> class QualitymOcf : public QualityOcf<Scalarm, T> {
		public:  static void Name(std::vector<std::string>& name) { name.push_back(std::string("QualitymOcf")); T::Name(name); }
		};

		template <class T> class CurvatureDirmOcf : public CurvatureDirOcf<CurvatureDirOcfBaseType<Scalarm>, T> {
		public:	static void Name(std::vector<std::string>& name) { name.push_back(std::string("CurvatureDirdOcf")); T::Name(name); }
		};

	}//end namespace face
}//end namespace vcg

// Forward declarations needed for creating the used types
class CVertexO;
class CEdgeO;
class CFaceO;

// Declaration of the semantic of the used types
class CUsedTypesO : public vcg::UsedTypes < vcg::Use<CVertexO>::AsVertexType,
	vcg::Use<CEdgeO   >::AsEdgeType,
	vcg::Use<CFaceO  >::AsFaceType > {};


// The Main Vertex Class
// Most of the attributes are optional and must be enabled before use.
// Each vertex needs 40 byte, on 32bit arch. and 44 byte on 64bit arch.

class CVertexO : public vcg::Vertex< CUsedTypesO,
	vcg::vertex::InfoOcf,           /*  4b */
	vcg::vertex::Coord3m,           /* 12b */
	vcg::vertex::BitFlags,          /*  4b */
	vcg::vertex::Normal3m,          /* 12b */
	vcg::vertex::Qualitym,          /*  4b */
	vcg::vertex::Color4b,           /*  4b */
	vcg::vertex::VFAdjOcf,          /*  0b */
	vcg::vertex::MarkOcf,           /*  0b */
	vcg::vertex::TexCoordfOcf,      /*  0b */
	vcg::vertex::CurvatureDirmOcf,  /*  0b */
	vcg::vertex::RadiusmOcf         /*  0b */
> {
};


// The Main Edge Class
class CEdgeO : public vcg::Edge<CUsedTypesO,
	vcg::edge::BitFlags,          /*  4b */
	vcg::edge::EVAdj,
	vcg::edge::EEAdj,
	vcg::edge::EFAdj
> {
};

// Each face needs 32 byte, on 32bit arch. and 48 byte on 64bit arch.
class CFaceO : public vcg::Face<  CUsedTypesO,
	vcg::face::InfoOcf,              /* 4b */
	vcg::face::VertexRef,            /*12b */
	vcg::face::BitFlags,             /* 4b */
	vcg::face::Normal3m,             /*12b */
	vcg::face::QualitymOcf,          /* 0b */
	vcg::face::MarkOcf,              /* 0b */
	vcg::face::Color4bOcf,           /* 0b */
	vcg::face::FFAdjOcf,             /* 0b */
	vcg::face::VFAdjOcf,             /* 0b */
	vcg::face::CurvatureDirmOcf,     /* 0b */
	vcg::face::WedgeTexCoordfOcf     /* 0b */
> {
};

typedef vcg::tri::TriMesh< vcg::vertex::vector_ocf<CVertexO>, vcg::face::vector_ocf<CFaceO> > vcgTriMesh;

class CMeshO : public vcgTriMesh
{
public:
	CMeshO();

	CMeshO(const CMeshO& oth);

	CMeshO(CMeshO&& oth);

	virtual ~CMeshO();

	CMeshO& operator=(CMeshO oth);

	friend void swap(CMeshO& m1, CMeshO& m2);

	Box3m trBB() const;

	int sfn;    //The number of selected faces.
	int svn;    //The number of selected vertices.

	int pvn; //the number of the polygonal vertices
	int pfn; //the number of the polygonal faces 

	Matrix44m Tr; // Usually it is the identity. It is applied in rendering and filters can or cannot use it. (most of the filter will ignore this)
	
	void UnMarkAll() { vcg::tri::UnMarkAll(*this); }

	bool IsMarked(CFaceO* obj) { return (vcg::tri::IsMarked(*this, obj)); }

	void Mark(CFaceO* obj) { vcg::tri::Mark(*this, obj); }

	void updateBoxAndNormals()
	{
		vcg::tri::UpdateBounding<CMeshO>::Box(*this);
		if (this->fn > 0) {
			vcg::tri::UpdateNormal<CMeshO>::PerFaceNormalized(*this);
			vcg::tri::UpdateNormal<CMeshO>::PerVertexAngleWeighted(*this);
		}
	}

	void updateDataMask()
	{
		this->face.EnableFFAdjacency();
		vcg::tri::UpdateTopology<CMeshO>::FaceFace(*this);

		this->vert.EnableVFAdjacency();
		this->face.EnableVFAdjacency();
		vcg::tri::UpdateTopology<CMeshO>::VertexFace(*this);

		this->face.EnableWedgeTexCoord();
		this->face.EnableColor();
		this->face.EnableQuality();
		this->face.EnableCurvatureDir();
		this->face.EnableMark();
		this->vert.EnableMark();
		this->vert.EnableCurvatureDir();
		this->vert.EnableRadius();
		this->vert.EnableTexCoord();
	}

	MyMesh toOpenMesh()
	{
		MyMesh openMesh;
		std::vector<MyMesh::VertexHandle> vhandles;
		for (size_t i = 0; i < this->vert.size(); ++i)
		{
			if (!this->vert[i].IsD())
			{
				auto vh = openMesh.add_vertex(OpenMesh::Vec3f(this->vert[i].P()[0],
					this->vert[i].P()[1],
					this->vert[i].P()[2]));
				vhandles.push_back(vh);
			}
		}

		for (size_t i = 0; i < this->face.size(); ++i)
		{
			if (!this->face[i].IsD())
			{
				std::vector<MyMesh::VertexHandle> face_vhandles;
				for (int j = 0; j < 3; ++j)
				{
					int index = vcg::tri::Index(*this, this->face[i].V(j));
					face_vhandles.push_back(vhandles[index]);
				}
				openMesh.add_face(face_vhandles);
			}
		}

		openMesh.update_normals();
		return openMesh;
	}

	void EnableAttribute()
	{
		this->face.EnableFFAdjacency();
		this->vert.EnableVFAdjacency();
		this->face.EnableVFAdjacency();
		this->face.EnableWedgeTexCoord();
		this->face.EnableColor();
		this->face.EnableQuality();
		this->face.EnableCurvatureDir();
		this->face.EnableMark();
		this->vert.EnableMark();
		this->vert.EnableCurvatureDir();
		this->vert.EnableRadius();
		this->vert.EnableTexCoord();
		this->face.EnableFFAdjacency();
		this->vert.EnableVFAdjacency();
		vcg::tri::InitFaceIMark(*this);
		vcg::tri::UnMarkAll(*this);
		vcg::tri::Clean<CMeshO>::RemoveDuplicateVertex(*this);
		vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(*this);
		vcg::tri::Allocator<CMeshO>::CompactEveryVector(*this);
		vcg::tri::UpdateNormal<CMeshO>::PerFaceNormalized(*this);
		vcg::tri::UpdateNormal<CMeshO>::PerVertexFromCurrentFaceNormal(*this);
		vcg::tri::UpdateBounding<CMeshO>::Box(*this);
		vcg::tri::UpdateTopology<CMeshO>::FaceFace(*this);
		if (this->fn > 0) {
			vcg::tri::UpdateNormal<CMeshO>::PerFaceNormalized(*this);
			vcg::tri::UpdateNormal<CMeshO>::PerVertexAngleWeighted(*this);
		}
	}

private:
	void enableComponentsFromOtherMesh(const CMeshO& oth);
};

inline void swap(CMeshO& m1, CMeshO& m2)
{
	using std::swap;
	swap(m1.vn, m2.vn);
	swap(m1.vert, m2.vert);
	m1.vert._updateOVP(m1.vert.begin(), m1.vert.end());
	m2.vert._updateOVP(m2.vert.begin(), m2.vert.end());
	swap(m1.en, m2.en);
	swap(m1.edge, m2.edge);
	swap(m1.fn, m2.fn);
	swap(m1.face, m2.face);
	m1.face._updateOVP(m1.face.begin(), m1.face.end());
	m2.face._updateOVP(m2.face.begin(), m2.face.end());
	swap(m1.hn, m2.hn);
	swap(m1.hedge, m2.hedge);
	swap(m1.tn, m2.tn);
	swap(m1.tetra, m2.tetra);
	swap(m1.bbox, m2.bbox);
	swap(m1.textures, m2.textures);
	swap(m1.normalmaps, m2.normalmaps);
	swap(m1.attrn, m2.attrn);
	swap(m1.vert_attr, m2.vert_attr);
	swap(m1.edge_attr, m2.edge_attr);
	swap(m1.face_attr, m2.face_attr);
	swap(m1.mesh_attr, m2.mesh_attr);
	swap(m1.tetra_attr, m2.tetra_attr);
	swap(m1.shot, m2.shot);
	swap(m1.imark, m2.imark);
}
#endif