#ifndef CUTMESH_H
#define CUTMESH_H
#include "mymesh.h"
typedef vcg::Point3f Point3f;
class CutMesh 
{
    struct MeshEdge
    {
        int v0, v1;

        MeshEdge(int a, int b) {
            v0 = std::min(a, b);
            v1 = std::max(a, b);
        }

        bool operator<(const MeshEdge& other) const {
            return std::tie(v0, v1) < std::tie(other.v0, other.v1);
        }
    };
public:
    int getEdgeValue(std::map<MeshEdge, int> edge_map, const MeshEdge& edge)
    {
        auto it = edge_map.find(edge);
        return (it != edge_map.end()) ? it->second : -1;
    }

    static bool IntersectPlaneSegment(Point3f pN, Point3f pC, Point3f sP, Point3f eP, Point3f& intersection)
    {
        Point3f dir = eP - sP;
        float denom = pN.dot(dir);
        if (fabs(denom) < std::numeric_limits<float>::epsilon())
            return false;  // 平行情况
        float t = -((pN.dot(sP) - pN.dot(pC)) / denom);
        if (t < 0.0 || t > 1.0)
            return false;  // 交点不在线段内
        intersection = sP + dir * t;
        return true;
    }

    static void judgeIsAddPoint(CMeshO& mesh, std::map<MeshEdge, int>& edge_map, int v0, int v1, Point3f planeDir, Point3f planeCenter, int& new_id)
    {
        MeshEdge cur_edge = MeshEdge(v0, v1);
        auto it = edge_map.find(cur_edge);
        if (edge_map.find(cur_edge) != edge_map.end())
        {
            new_id = edge_map[cur_edge];
        }
        else
        {
            Point3f intersection;
            CVertexO* vS = mesh.vert.data() + v0;
            CVertexO* vE = mesh.vert.data() + v1;
            if (IntersectPlaneSegment(planeDir, planeCenter, vS->P(), vE->P(), intersection))
            {
                CMeshO::VertexIterator pv = vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 1);
                pv[0].P().Import(intersection);
                new_id = vcg::tri::Index(mesh, pv[0]);
                edge_map[cur_edge] = new_id;
            }
        }
    }

    static int RemoveUnreferencedVertex(CMeshO& m, bool DeleteVertexFlag = true)   // V1.0
    {
        vcg::tri::RequirePerVertexFlags(m);

        std::vector<bool> referredVec(m.vert.size(), false);
        int deleted = 0;

        for (auto fi = m.face.begin(); fi != m.face.end(); ++fi)
            if (!(*fi).IsD())
                for (auto j = 0; j < (*fi).VN(); ++j)
                    referredVec[vcg::tri::Index(m, (*fi).V(j))] = true;

        for (auto ei = m.edge.begin(); ei != m.edge.end(); ++ei)
            if (!(*ei).IsD()) {
                referredVec[vcg::tri::Index(m, (*ei).V(0))] = true;
                referredVec[vcg::tri::Index(m, (*ei).V(1))] = true;
            }

        for (auto ti = m.tetra.begin(); ti != m.tetra.end(); ++ti)
            if (!(*ti).IsD()) {
                referredVec[vcg::tri::Index(m, (*ti).V(0))] = true;
                referredVec[vcg::tri::Index(m, (*ti).V(1))] = true;
                referredVec[vcg::tri::Index(m, (*ti).V(2))] = true;
                referredVec[vcg::tri::Index(m, (*ti).V(3))] = true;
            }


        if (!DeleteVertexFlag)
            return std::count(referredVec.begin(), referredVec.end(), false);

        for (auto vi = m.vert.begin(); vi != m.vert.end(); ++vi)
            if ((!(*vi).IsD()) && (!referredVec[vcg::tri::Index(m, *vi)]))
            {
                vcg::tri::Allocator<CMeshO>::DeleteVertex(m, *vi);
                ++deleted;
            }
        return deleted;
    }

    static void PlaneCutMesh(Point3f planeDir, Point3f planeCenter, CMeshO& originMesh, std::pair<CMeshO, CMeshO>& result)
    {
        CMeshO inputMesh = originMesh;
        vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<int>(inputMesh, "VertexValue");
        CMeshO::PerVertexAttributeHandle<int> vertex_Attr = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<int>(inputMesh, "VertexValue");

        float proDist = 0.0;
        Point3f proDir = planeDir.Normalize();
        for (int i = 0; i < inputMesh.vn; i++)
        {
            CVertexO& v_it = inputMesh.vert[i];
            proDist = (v_it.P() - planeCenter) * proDir;
            vertex_Attr[i] = (proDist > 1e-5) ? 1 : (proDist < -1e-5 ? -1 : 0);

            CMeshO::VertexIterator v_left = vcg::tri::Allocator<CMeshO>::AddVertices(result.first, 1);
            CMeshO::VertexIterator v_right = vcg::tri::Allocator<CMeshO>::AddVertices(result.second, 1);
            (*v_left).P().Import(v_it.P());
            (*v_right).P().Import(v_it.P());
        }

        std::map<MeshEdge, int> edge_map_left;
        std::map<MeshEdge, int> edge_map_right;
        for (const CFaceO& f_it : inputMesh.face)
        {
            std::vector<int> leftVerts, rightVerts, midVerts;

            // 统计面顶点位置
            for (int j = 0; j < 3; ++j)
            {
                int idx = vcg::tri::Index(inputMesh, f_it.V(j));
                if (vertex_Attr[idx] == 1) leftVerts.push_back(idx);
                else if (vertex_Attr[idx] == -1) rightVerts.push_back(idx);
                else midVerts.push_back(idx);
            }

            // 完全在一侧
            if (leftVerts.size() == 3)
            {
                auto fi = vcg::tri::Allocator<CMeshO>::AddFace(result.first, leftVerts[0], leftVerts[1], leftVerts[2]);
            }
            else if (rightVerts.size() == 3)
            {
                vcg::tri::Allocator<CMeshO>::AddFace(result.second, rightVerts[0], rightVerts[1], rightVerts[2]);
            }
            else
            {
                std::vector<int> sideCount[2] = { leftVerts, rightVerts };
                if (sideCount[0].size() == 2 && sideCount[1].size() == 1)
                {
                    // 情况 1：两个点在左，一点在右
                    int idxA = sideCount[0][0], idxB = sideCount[0][1], idxC = sideCount[1][0];
                    bool isSwap = false;
                    for (int j = 0; j < 3; j++)
                    {
                        int cur_idx = vcg::tri::Index(inputMesh, f_it.V(j));
                        int next_idx = vcg::tri::Index(inputMesh, f_it.V((j + 1) % 3));
                        if (cur_idx == idxA && next_idx == idxC)
                        {
                            isSwap = true;
                            break;
                        }
                    }

                    int v0_left = -1;
                    int v1_left = -1;
                    int v0_right = -1;
                    int v1_right = -1;
                    judgeIsAddPoint(result.first, edge_map_left, idxA, idxC, proDir, planeCenter, v0_left);
                    judgeIsAddPoint(result.first, edge_map_left, idxC, idxB, proDir, planeCenter, v1_left);
                    judgeIsAddPoint(result.second, edge_map_right, idxA, idxC, proDir, planeCenter, v0_right);
                    judgeIsAddPoint(result.second, edge_map_right, idxC, idxB, proDir, planeCenter, v1_right);
                    if (isSwap)
                    {
                        vcg::tri::Allocator<CMeshO>::AddFace(result.first, idxB, idxA, v0_left);
                        vcg::tri::Allocator<CMeshO>::AddFace(result.first, idxB, v0_left, v1_left);
                        vcg::tri::Allocator<CMeshO>::AddFace(result.second, idxC, v1_right, v0_right);
                    }
                    else
                    {
                        vcg::tri::Allocator<CMeshO>::AddFace(result.first, idxA, idxB, v0_left);
                        vcg::tri::Allocator<CMeshO>::AddFace(result.first, idxB, v1_left, v0_left);
                        vcg::tri::Allocator<CMeshO>::AddFace(result.second, idxC, v0_right, v1_right);
                    }
                }
                else if (sideCount[1].size() == 2 && sideCount[0].size() == 1)
                {
                    //两个点在右，一点在左
                    int idxA = sideCount[0][0], idxB = sideCount[1][0], idxC = sideCount[1][1];
                    bool isSwap = false;
                    for (int j = 0; j < 3; j++)
                    {
                        int cur_idx = vcg::tri::Index(inputMesh, f_it.V(j));
                        int next_idx = vcg::tri::Index(inputMesh, f_it.V((j + 1) % 3));
                        if (cur_idx == idxA && next_idx == idxC)
                        {
                            isSwap = true;
                            break;
                        }
                    }
                    int v0_left = -1;
                    int v1_left = -1;
                    int v0_right = -1;
                    int v1_right = -1;
                    judgeIsAddPoint(result.first, edge_map_left, idxA, idxB, proDir, planeCenter, v0_left);
                    judgeIsAddPoint(result.first, edge_map_left, idxC, idxA, proDir, planeCenter, v1_left);
                    judgeIsAddPoint(result.second, edge_map_right, idxA, idxB, proDir, planeCenter, v0_right);
                    judgeIsAddPoint(result.second, edge_map_right, idxC, idxA, proDir, planeCenter, v1_right);
                    if (isSwap)
                    {
                        vcg::tri::Allocator<CMeshO>::AddFace(result.first, idxA, v1_left, v0_left);
                        vcg::tri::Allocator<CMeshO>::AddFace(result.second, v0_right, idxC, idxB);
                        vcg::tri::Allocator<CMeshO>::AddFace(result.second, idxC, v0_right, v1_right);
                    }
                    else
                    {
                        vcg::tri::Allocator<CMeshO>::AddFace(result.first, idxA, v0_left, v1_left);
                        vcg::tri::Allocator<CMeshO>::AddFace(result.second, v0_right, idxB, idxC);
                        vcg::tri::Allocator<CMeshO>::AddFace(result.second, idxC, v1_right, v0_right);
                    }
                }
                else if (midVerts.size() == 2)
                {
                    //两个点在平面上
                    if (sideCount[0].size() == 1)
                    {
                        vcg::tri::Allocator<CMeshO>::AddFace(result.first, midVerts[0], midVerts[1], sideCount[0][0]);
                    }
                    else
                    {
                        vcg::tri::Allocator<CMeshO>::AddFace(result.second, midVerts[0], midVerts[1], sideCount[1][0]);
                    }
                }
                else if (midVerts.size() == 1)
                {

                    if (sideCount[0].size() == 2)
                    {
                        vcg::tri::Allocator<CMeshO>::AddFace(result.first, midVerts[0], sideCount[0][0], sideCount[0][1]);
                    }
                    else if (sideCount[1].size() == 2)
                    {
                        vcg::tri::Allocator<CMeshO>::AddFace(result.second, midVerts[0], sideCount[1][0], sideCount[1][1]);
                    }
                    else if (sideCount[0].size() == 1 && sideCount[1].size() == 1)
                    {
                        int idxA = midVerts[0], idxB = sideCount[0][0], idxC = sideCount[1][0];
                        bool isSwap = false;
                        for (int j = 0; j < 3; j++)
                        {
                            int cur_idx = vcg::tri::Index(inputMesh, f_it.V(j));
                            int next_idx = vcg::tri::Index(inputMesh, f_it.V((j + 1) % 3));
                            if (cur_idx == idxA && next_idx == idxC)
                            {
                                isSwap = true;
                                break;
                            }
                        }

                        int v0_left = -1;
                        int v0_right = -1;
                        judgeIsAddPoint(result.first, edge_map_left, idxB, idxC, proDir, planeCenter, v0_left);
                        judgeIsAddPoint(result.second, edge_map_right, idxB, idxC, proDir, planeCenter, v0_right);
                        if (isSwap)
                        {
                            vcg::tri::Allocator<CMeshO>::AddFace(result.first, midVerts[0], v0_left, sideCount[0][0]);
                            vcg::tri::Allocator<CMeshO>::AddFace(result.second, midVerts[0], sideCount[1][0], v0_right);
                        }
                        else
                        {
                            vcg::tri::Allocator<CMeshO>::AddFace(result.first, midVerts[0], sideCount[0][0], v0_left);
                            vcg::tri::Allocator<CMeshO>::AddFace(result.second, midVerts[0], v0_right, sideCount[1][0]);
                        }
                    }
                }
            }
        }

        result.first.vert.EnableVFAdjacency();
        result.first.face.EnableVFAdjacency();
        result.second.vert.EnableVFAdjacency();
        result.second.face.EnableVFAdjacency();
        RemoveUnreferencedVertex(result.first);
        RemoveUnreferencedVertex(result.second);
        vcg::tri::Allocator<CMeshO>::CompactVertexVector(result.first);
        vcg::tri::Allocator<CMeshO>::CompactVertexVector(result.second);
    }
};
#endif
