#ifndef SWEEPING_H
#define SWEEPING_H
#include"mymesh.h"
//#include <wrap/io_trimesh/export_obj.h>

typedef vcg::Point3f Point3f;
class SweepingMesh
{
public:
	static void GenerateSweptVolume(const Point3f& moveDir, float moveDist, CMeshO& mesh, std::vector<MyMesh::VertexHandle> boundaryPts)
	{
#if 0
		CMeshO resultMesh;
		std::map<int, int> point_map;
		for (int i = 0; i < mesh.vn; i++)
		{
			CMeshO::VertexIterator vIter = vcg::tri::Allocator<CMeshO>::AddVertex(resultMesh, mesh.vert[i].P());
			point_map[vcg::tri::Index(mesh, mesh.vert[i])] = vcg::tri::Index(resultMesh, *vIter);
		}
		for (int i = 0; i < resultMesh.vn; i++)
		{
			std::cout << " index: " << vcg::tri::Index(resultMesh, resultMesh.vert[i]) << std::endl;
		}
		for (int i = 0; i < mesh.fn; i++)
		{
			int newV[3];
			for (int j = 0; j < 3; j++)
			{
				newV[j] = point_map[vcg::tri::Index(mesh, mesh.face[i].V(j))];
			}
			vcg::tri::Allocator<CMeshO>::AddFace(resultMesh, newV[0], newV[1], newV[2]);
		}
#endif
		int count = boundaryPts.size();
		/*float avrage_length = 0.0;
		for (int i = 0; i < count; i++)
		{
			avrage_length += (boundaryPts[i].P() - boundaryPts[(i + 1) % count].P()).Norm();
		}
		avrage_length /= count;*/


		float avrage_length = 0.125;
		int num = mesh.vn;
		int add_count = (int)(moveDist / avrage_length);
		avrage_length = moveDist / add_count;
		for (int j = 0; j < add_count; j++)
		{
			for (int i = 0; i < count; i++)
			{
				CVertexO* cv = mesh.vert.data() + boundaryPts[i].idx();
				Point3f addP = cv->P() + moveDir * avrage_length * (j + 1);
				CMeshO::VertexIterator vIter = vcg::tri::Allocator<CMeshO>::AddVertex(mesh, addP);
				//std::cout << "Vertex " << i << " index: " << vcg::tri::Index(mesh, *vIter) << std::endl;
			}
		}
		for (int j = 0; j < add_count - 1; j++)
		{
			for (int i = 0; i < count; i++)
			{
				int v0 = num + j * count + i;
				int v1 = num + (j + 1) * count + i;
				int v2 = num + (j + 1) * count + (i + 1) % count;
				//std::cout << "face vertex if: " << v0 << "," << v1 << "," << v2 << std::endl;
				vcg::tri::Allocator<CMeshO>::AddFace(mesh, num + j * count + i, num + (j + 1) * count + i, num + (j + 1) * count + (i + 1) % count);
				vcg::tri::Allocator<CMeshO>::AddFace(mesh, num + j * count + i, num + (j + 1) * count + (i + 1) % count, num + j * count + (i + 1) % count);
			}
		}
		for (int i = 0; i < count; i++)
		{
			int cur_idx = boundaryPts[i].idx();
			int next_idx = boundaryPts[(i + 1) % count].idx();
			vcg::tri::Allocator<CMeshO>::AddFace(mesh, cur_idx, num + i, num + (i + 1) % count);
			vcg::tri::Allocator<CMeshO>::AddFace(mesh, cur_idx, num + (i + 1) % count, next_idx);
		}
		//std::string output_name = "D:/data/ConvexHull/SweptVolume.obj";
		//vcg::tri::io::ExporterOBJ<CMeshO>::Save(mesh, output_name.c_str(), 0);
	}
};
#endif
