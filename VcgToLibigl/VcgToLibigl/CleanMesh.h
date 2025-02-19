#ifndef CLEAN_MESH_H
#define CLEAN_MESH_H
#include "mymesh.h"
using namespace vcg;

#include <vcg/complex/complex.h>
#include <vcg/complex/base.h>
#include<vcg/simplex/face/pos.h>
#include <queue>
#include <map>
#include <set>
#include <vcg/simplex/vertex/base.h>   
#include <vcg/simplex/vertex/component.h>   
#include <vcg/simplex/face/base.h>   
#include <vcg/simplex/face/component.h>   

#include <vcg/complex/complex.h>   
#include<vcg/complex/algorithms/create/platonic.h>

#include<vcg/complex/algorithms/update/topology.h>
#include <wrap/io_trimesh/export_obj.h>
class CleanMesh
{
public:
	static int GetVertexRingVertices(CVertexO* v, std::vector<CVertexO*>& ringVertics)
	{
		//v = &(mesh->vert[0]);
		std::set<int> visited;
		vcg::face::VFIterator<CFaceO> vfi(v);
		for (; !vfi.End(); ++vfi)
		{
			CFaceO* f = vfi.F();
			auto v0 = f->V((vfi.z + 1) % 3);
			auto v1 = f->V((vfi.z + 2) % 3);
			if (visited.find(v0->Index()) == visited.end())
			{
				visited.insert(v0->Index());
				ringVertics.push_back(v0);
			}
			if (visited.find(v1->Index()) == visited.end())
			{
				visited.insert(v1->Index());
				ringVertics.push_back(v1);
			}
		}
		return ringVertics.size();
	}

	static std::vector<std::vector<int>> getAllConnectedVertices(CMeshO& mesh)
	{
		if (mesh.vn <= 3)
			return {};
		for (auto& v_it : mesh.vert)
		{
			v_it.ClearV();
		}
		vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
		std::vector<std::vector<int>> verVertexList;
		for (int i = 0; i < mesh.vn; i++)
		{
			if (mesh.vert[i].IsV())
				continue;

			int seed = i;
			std::vector<int> curVertexList;
			CVertexO& v0 = mesh.vert[seed];
			v0.SetV();
			std::queue<int> ringV;
			curVertexList.push_back(seed);
			ringV.push(seed);
			while (!ringV.empty())
			{
				int curIndex = ringV.front();
				CVertexO* v = &(mesh.vert[curIndex]);
				ringV.pop();

				std::vector<CVertexO*> ringVertex;
				GetVertexRingVertices(v, ringVertex);
				for (auto itor : ringVertex)
				{
					if (itor->IsV())
						continue;
					itor->SetV();
					ringV.push(vcg::tri::Index(mesh, itor));
					curVertexList.push_back(vcg::tri::Index(mesh, itor));
				}
			}
			verVertexList.push_back(curVertexList);
		}
		return verVertexList;
	}

	static void getAllConnectedComponent(CMeshO& mesh, std::vector<CMeshO>& resultMeshlist)
	{
		mesh.EnableAttribute();
		std::vector<std::vector<int>> verVertexList = getAllConnectedVertices(mesh);
		if (verVertexList.size() == 0)
			return;

		for (int i = 0; i < verVertexList.size(); i++)
		{
			for (auto& v_it : mesh.vert)
			{
				v_it.ClearV();
			}
			CMeshO resultMesh;
			std::vector<CFaceO> selectFaces;
			std::map<int, int> indexMap;
			std::set<int> curVerSet(verVertexList[i].begin(), verVertexList[i].end());
			int newIndex = -1;
			for (auto& f : mesh.face)
			{
				bool allVerticesInSet = true;

				for (int k = 0; k < 3; k++)
				{
					if (curVerSet.find(vcg::tri::Index(mesh, f.V(k))) == curVerSet.end())
					{
						allVerticesInSet = false;
						break;
					}
				}
				if (allVerticesInSet)
				{
					for (int k = 0; k < 3; k++)
					{
						CVertexO* curV = f.V(k);
						if (curV->IsV())
							continue;
						curV->SetV();
						auto add_v = vcg::tri::Allocator<CMeshO>::AddVertices(resultMesh, 1);
						(*add_v).P().Import(curV->P());
						newIndex++;
						indexMap[vcg::tri::Index(mesh, f.V(k))] = newIndex;
					}
					selectFaces.push_back(f);
				}
			}

			for (auto& f : selectFaces)
			{
				int old_v0 = vcg::tri::Index(mesh, f.V(0));
				int old_v1 = vcg::tri::Index(mesh, f.V(1));
				int old_v2 = vcg::tri::Index(mesh, f.V(2));
				auto add_v = vcg::tri::Allocator<CMeshO>::AddFace(resultMesh, indexMap[old_v0], indexMap[old_v1], indexMap[old_v2]);
			}
			/*std::string output_name = "D:/data/ConvexHull/connectionMesh" + std::to_string(i) + ".obj";
			vcg::tri::io::ExporterOBJ<CMeshO>::Save(resultMesh, output_name.c_str(), 0);*/
			resultMeshlist.push_back(resultMesh);
		}
	}

	static void getMaxConnectedComponent(CMeshO& mesh, CMeshO& resultMesh)
	{
		std::vector<CMeshO> resultMeshlist;
		getAllConnectedComponent(mesh, resultMeshlist);
		std::sort(resultMeshlist.begin(), resultMeshlist.end(),
			[](const CMeshO& a, const CMeshO& b) {
				return a.vn > b.vn;
			});
		resultMeshlist.resize(1);
		resultMesh = resultMeshlist[0];
	}
};
#endif