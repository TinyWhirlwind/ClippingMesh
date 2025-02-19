#include "ConvexHull.h"
#include "mymesh.h"
#include "vcg/complex/allocate.h"
#include "wrap/io_trimesh/io_mask.h"
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/space/triangle3.h>
#include <wrap/io_trimesh/export_stl.h>
#include <wrap/io_trimesh/export_obj.h>
#include <queue>


//#include <vcg/space/ray3.h>
//#include <vcg/complex/algorithms/intersection.h>
//#include <vcg/space/box3.h>
//#include <vcg/space/line3.h>
//#include <vcg/space/index/grid_util.h>
//#include <vcg/space/index/grid_closest.h>
//#include <vcg/simplex/face/distance.h>
//#include <vcg/complex/algorithms/closest.h>
//#include <dirt_utils.h>
class QuickHull::PImpl
{
public:
	PImpl(CMeshO& mesh)
	{
		mesh_ = mesh;
	}
	~PImpl()
	{}

public:
	void apply(CMeshO& input_mesh, CMeshO& convex_hull_mesh);
	bool getInitConvexHull(CMeshO& input_mesh, CMeshO& convex_hull_mesh);
	void getRingFaces(CFaceO* cur_face, std::vector<CFaceO*>& neighbor_faces);
public:
	CMeshO mesh_;
	CMeshO convex_hull_;
	std::vector<CVertexO*> vertices_;
};

QuickHull::QuickHull(std::vector<CMeshO> mesh)
{
	CMeshO vert_mesh;
	for (int i = 0; i < mesh.size(); i++)
	{
		CMeshO::VertexIterator vIter = vcg::tri::Allocator<CMeshO>::AddVertices(vert_mesh, mesh[i].vert.size());
		for (int j = 0; j < mesh[i].vert.size(); j++)
		{
			(*vIter).P().Import(mesh[i].vert[j].P());
			(*vIter).N().Import(mesh[i].vert[j].N()); // 复制法线
			vIter++;
		}
	}
	vcg::tri::Allocator<CMeshO>::CompactEveryVector(vert_mesh);
	impl_.reset(new PImpl(vert_mesh));
}
QuickHull::~QuickHull()
{

}

void QuickHull::apply()
{
	impl_->apply(impl_->mesh_, impl_->convex_hull_);
}

void QuickHull::PImpl::apply(CMeshO& input_mesh, CMeshO& convex_hull)
{
	convex_hull.face.EnableFFAdjacency();
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(convex_hull);

	vcg::tri::RequireFFAdjacency(convex_hull);
	vcg::tri::RequirePerFaceNormal(convex_hull);
	vcg::tri::Allocator<CMeshO>::CompactVertexVector(input_mesh);
	typename CMeshO:: template PerVertexAttributeHandle<size_t> indexInputVertex = vcg::tri::Allocator<CMeshO>::template GetPerVertexAttribute<size_t>(convex_hull, std::string("indexInput"));
	if (input_mesh.vert.size() < 4)
		return;
	vcg::tri::UpdateFlags<CMeshO>::VertexClearV(input_mesh);
	getInitConvexHull(input_mesh, convex_hull);

	std::vector<std::vector<CMeshO::VertexPointer>> undetermined_faces(convex_hull.face.size());//待定面集Q
	std::vector<std::pair<CMeshO::VertexPointer, double>> ver_face_furth_point(convex_hull.face.size(), std::make_pair(nullptr, 0.0f));
	for (int i = 0; i < input_mesh.vert.size(); ++i)
	{
		if (input_mesh.vert[i].IsV())
			continue;
		for (int j = 0; j < convex_hull.fn; ++j)
		{
			CFaceO fi = convex_hull.face[j];
			double dist = (input_mesh.vert[i].P() - fi.V(0)->P()) * fi.N();
			if (dist > 0.0f)
			{
				undetermined_faces[j].emplace_back(&input_mesh.vert[i]);
				if (dist > ver_face_furth_point[j].second)
				{
					ver_face_furth_point[j].second = dist;
					ver_face_furth_point[j].first = &input_mesh.vert[i];
				}
			}
		}
	}


	for (int i = 0; i < undetermined_faces.size(); i++)
	{
		if (undetermined_faces[i].size() == 0)
			continue;
		CMeshO::VertexPointer furth_point = ver_face_furth_point[i].first;
		std::queue<int> queue;
		std::vector<int> border_faces;
		std::vector<int> visit_faces;
		visit_faces.push_back(i);
		queue.push(i);
		while (queue.size() > 0)
		{
			CMeshO::FacePointer fp = &convex_hull.face[queue.front()];
			queue.pop();
			fp->SetV();
			for (int j = 0; j < 3; j++)
			{
				CMeshO::FacePointer nextF = fp->FFp(j);
				if (nextF->IsV())
					continue;
				int face_index = vcg::tri::Index(convex_hull, nextF);
				double dist = (furth_point->P() - nextF->P(0)) * nextF->N();
				if (dist < 0.0f)
				{
					border_faces.push_back(face_index);
					fp->SetB(j);
					nextF->SetB(fp->FFi(j));//当前面的第 j 条边对应的相邻面中，与该边共享的边的索引
				}
				else
				{
					visit_faces.push_back(face_index);
					queue.push(face_index);
				}
			}
		}

		if (border_faces.size() > 0)
		{
			CMeshO::VertexIterator vi = vcg::tri::Allocator<CMeshO>::AddVertices(convex_hull, 1);
			(*vi).P().Import((*furth_point).P());
			furth_point->SetV();
			indexInputVertex[vi] = vcg::tri::Index(input_mesh, furth_point);
		}

		std::unordered_map<CMeshO::VertexPointer, std::pair<int, char> > fanMap;//记录边和面片对应关系
		for (int j = 0; j < border_faces.size(); j++)
		{
			int face_index = border_faces[j];
			CMeshO::FacePointer fp = &convex_hull.face[face_index];
			for (int k = 0; k < 3; k++)
			{
				if (fp->IsB(k))
				{
					fp->ClearB(k);
					CMeshO::FaceIterator fi = vcg::tri::Allocator<CMeshO>::AddFace(convex_hull, &convex_hull.vert.back(), fp->V1(k), fp->V0(k));//fp的第j条边的from和to
					(*fi).N() = vcg::NormalizedTriangleNormal(*fi);
					fp = &convex_hull.face[face_index];
					int new_face_index = vcg::tri::Index(convex_hull, *fi);
					//Update convex hull FF topology
					CMeshO::VertexPointer vp[] = { fp->V1(k),fp->V0(k) };
					for (int n = 0; n < 2; n++)
					{
						int indexE = n * 2;
						typename std::unordered_map<CMeshO::VertexPointer, std::pair<int, char> >::iterator vIter = fanMap.find(vp[n]);
						if (vIter != fanMap.end())
						{
							//若一个面片有多个边界边 更新面片和相邻面片的领接关系
							CMeshO::FacePointer f2 = &convex_hull.face[(*vIter).second.first];
							char edgeIndex = (*vIter).second.second;
							f2->FFp(edgeIndex) = &convex_hull.face.back();
							f2->FFi(edgeIndex) = indexE;
							fi->FFp(indexE) = f2;
							fi->FFi(indexE) = edgeIndex;
						}
						else
						{
							fanMap[vp[n]] = std::make_pair(new_face_index, indexE);
						}
					}

					//构建可见性列表，并找到该面所能看到的最远顶点
					std::vector<CMeshO::VertexPointer> tempVect;
					int indices[2] = { face_index,int(vcg::tri::Index(convex_hull,fp->FFp(k))) };
					std::vector<CMeshO::VertexPointer> vertexToTest(undetermined_faces[indices[0]].size() + undetermined_faces[indices[1]].size());
					typename std::vector<CMeshO::VertexPointer>::iterator tempIt = std::set_union(undetermined_faces[indices[0]].begin(), undetermined_faces[indices[0]].end(), undetermined_faces[indices[1]].begin(), undetermined_faces[indices[1]].end(), vertexToTest.begin());
					vertexToTest.resize(tempIt - vertexToTest.begin());

					std::pair<CMeshO::VertexPointer, double> newInfo = std::make_pair((CMeshO::VertexPointer)NULL, 0.0f);
					for (int n = 0; n < vertexToTest.size(); n++)
					{
						if (vertexToTest[n]->IsV())continue;
						float dist = (vertexToTest[n]->P() - fi->P(0)) * fi->N();
						if (dist > 0.0f)
						{
							tempVect.push_back(vertexToTest[n]);
							if (dist > newInfo.second)
							{
								newInfo.second = dist;
								newInfo.first = vertexToTest[n];
							}
						}
					}
					undetermined_faces.push_back(tempVect);
					ver_face_furth_point.push_back(newInfo);

					CMeshO::FacePointer ffp = fp->FFp(k);
					int ffi = fp->FFi(k);
					ffp->FFp(ffi) = ffp;
					ffp->FFi(ffi) = ffi;
					fp->FFp(k) = &convex_hull.face.back();
					fp->FFi(k) = 1;
					fi->FFp(1) = fp;
					fi->FFi(1) = k;
				}
			}
		}

		//删除内部面片 更新凸包
		for (int j = 0; j < visit_faces.size(); j++)
		{
			if (convex_hull.face[visit_faces[j]].IsD())continue;
			std::vector<CMeshO::VertexPointer> empty_vertex;
			vcg::tri::Allocator<CMeshO>::DeleteFace(convex_hull, convex_hull.face[visit_faces[j]]);
			undetermined_faces[visit_faces[j]].swap(empty_vertex);
		}
	}
	vcg::tri::UpdateTopology<CMeshO>::ClearFaceFace(convex_hull);
	vcg::tri::Allocator<CMeshO>::CompactFaceVector(convex_hull);
	vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(convex_hull);

	std::string output_name = "D:\\data\\ConvexHull\\ConvexHull.STL";
	vcg::tri::io::ExporterSTL<CMeshO>::Save(convex_hull, output_name.c_str());
}



//biggest tetraedron
bool QuickHull::PImpl::getInitConvexHull(CMeshO& input_mesh, CMeshO& convex_hull_mesh)
{
	typename CMeshO:: template PerVertexAttributeHandle<size_t> indexInputVertex = vcg::tri::Allocator<CMeshO>::template GetPerVertexAttribute<size_t>(convex_hull_mesh, std::string("indexInput"));
	CMeshO::VertexPointer v[3];
	//Find the 6 points with min/max coordinate values
	CMeshO::VertexIterator vi = input_mesh.vert.begin();
	std::vector<CMeshO::VertexPointer> minMax(6, &(*vi));
	for (; vi != input_mesh.vert.end(); vi++)
	{
		if ((*vi).P().X() < (*minMax[0]).P().X())
			minMax[0] = &(*vi);
		if ((*vi).P().Y() < (*minMax[1]).P().Y())
			minMax[1] = &(*vi);
		if ((*vi).P().Z() < (*minMax[2]).P().Z())
			minMax[2] = &(*vi);
		if ((*vi).P().X() > (*minMax[3]).P().X())
			minMax[3] = &(*vi);
		if ((*vi).P().Y() > (*minMax[4]).P().Y())
			minMax[4] = &(*vi);
		if ((*vi).P().Z() > (*minMax[5]).P().Z())
			minMax[5] = &(*vi);
	}
	//Find the farthest two points
	double maxDist = 0;
	for (int i = 0; i < 6; i++)
	{
		for (int j = i + 1; j < 6; j++)
		{
			float dist = (minMax[i]->P() - minMax[j]->P()).SquaredNorm();
			if (dist > maxDist)
			{
				maxDist = dist;
				v[0] = minMax[i];
				v[1] = minMax[j];
			}
		}
	}
	//Find the third point to create the base of the tetrahedron
	vcg::Line3<Scalarm> line(v[0]->P(), (v[0]->P() - v[1]->P()));
	maxDist = 0;
	for (vi = input_mesh.vert.begin(); vi != input_mesh.vert.end(); vi++)
	{
		Scalarm dist = vcg::Distance(line, (*vi).P());
		if (dist > maxDist)
		{
			maxDist = dist;
			v[2] = &(*vi);
		}
	}
	//Create face in the convex hull
	CMeshO::VertexIterator chVi = vcg::tri::Allocator<CMeshO>::AddVertices(convex_hull_mesh, 3);
	for (int i = 0; i < 3; i++)
	{
		(*chVi).P().Import(v[i]->P());
		v[i]->SetV();
		indexInputVertex[chVi] = vcg::tri::Index(input_mesh, v[i]);
		chVi++;
	}
	CMeshO::FaceIterator fi = vcg::tri::Allocator<CMeshO>::AddFace(convex_hull_mesh, 0, 1, 2);
	(*fi).N() = vcg::NormalizedTriangleNormal(*fi);

	//Find the fourth point to create the tetrahedron
	CMeshO::VertexPointer v4 = nullptr;
	float distance = 0;
	float absDist = -1;
	for (vi = input_mesh.vert.begin(); vi != input_mesh.vert.end(); vi++)
	{
		float tempDist = ((*vi).P() - (*fi).P(0)).dot((*fi).N());
		if (fabs(tempDist) > absDist)
		{
			distance = tempDist;
			v4 = &(*vi);
			absDist = fabs(distance);
		}
	}

	//Flip the previous face if the fourth point is above the face
	if (distance > 0)
	{
		(*fi).N() = -(*fi).N();
		CMeshO::VertexPointer tempV = (*fi).V(1);
		(*fi).V(1) = (*fi).V(2);
		(*fi).V(2) = tempV;
	}

	//Create the other 3 faces of the tetrahedron
	chVi = vcg::tri::Allocator<CMeshO>::AddVertices(convex_hull_mesh, 1);
	(*chVi).P().Import(v4->P());
	indexInputVertex[chVi] = vcg::tri::Index(input_mesh, v4);
	v4->SetV();
	fi = vcg::tri::Allocator<CMeshO>::AddFace(convex_hull_mesh, &convex_hull_mesh.vert[3], convex_hull_mesh.face[0].V0(1), convex_hull_mesh.face[0].V0(0));
	(*fi).N() = vcg::NormalizedTriangleNormal(*fi);
	fi = vcg::tri::Allocator<CMeshO>::AddFace(convex_hull_mesh, &convex_hull_mesh.vert[3], convex_hull_mesh.face[0].V1(1), convex_hull_mesh.face[0].V1(0));
	(*fi).N() = vcg::NormalizedTriangleNormal(*fi);
	fi = vcg::tri::Allocator<CMeshO>::AddFace(convex_hull_mesh, &convex_hull_mesh.vert[3], convex_hull_mesh.face[0].V2(1), convex_hull_mesh.face[0].V2(0));
	(*fi).N() = vcg::NormalizedTriangleNormal(*fi);
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(convex_hull_mesh);
	if (convex_hull_mesh.vn == 4)
		return true;
	return false;
}

void QuickHull::PImpl::getRingFaces(CFaceO* cur_face, std::vector<CFaceO*>& neighbor_faces)
{
	neighbor_faces.clear();
	for (int i = 0; i < 3; ++i)
	{
		CFaceO* neighbor = cur_face->FFp(i);
		if (neighbor != nullptr && !neighbor->IsV())
		{
			neighbor_faces.emplace_back(neighbor);
		}
	}
}

float QuickHull::getArea()
{
	return 0;
}

float QuickHull::getVolume()
{
	return 0;
}

CMeshO QuickHull::getMesh()
{
	return impl_->convex_hull_;
}

