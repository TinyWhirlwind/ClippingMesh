#include <vcg/space/point2.h>
#include "mymesh.h"
class BaseSampler
{
public:
    BaseSampler(CMeshO* _m) {
        m = _m;
        uvSpaceFlag = false;
        qualitySampling = false;
        perFaceNormal = false;
        //tex = 0;
    }
    CMeshO* m;
    //QImage* tex;
    int texSamplingWidth;
    int texSamplingHeight;
    bool uvSpaceFlag;
    bool qualitySampling;
    bool perFaceNormal;  // default false; if true the sample normal is the face normal, otherwise it is interpolated

    void reset()
    {
        m->Clear();
    }

    void AddVert(const CMeshO::VertexType& p)
    {
        vcg::tri::Allocator<CMeshO>::AddVertices(*m, 1);
        m->vert.back().ImportData(p);
    }

    void AddFace(const CMeshO::FaceType& f, CMeshO::CoordType p)
    {
        vcg::tri::Allocator<CMeshO>::AddVertices(*m, 1);
        m->vert.back().P() = f.cP(0) * p[0] + f.cP(1) * p[1] + f.cP(2) * p[2];

        if (perFaceNormal) m->vert.back().N() = f.cN();
        else m->vert.back().N() = f.cV(0)->N() * p[0] + f.cV(1)->N() * p[1] + f.cV(2)->N() * p[2];
        if (qualitySampling)
            m->vert.back().Q() = f.cV(0)->Q() * p[0] + f.cV(1)->Q() * p[1] + f.cV(2)->Q() * p[2];
    }
    void AddTextureSample(const CMeshO::FaceType& f, const CMeshO::CoordType& p, const vcg::Point2i& tp, float edgeDist)
    {
        if (edgeDist != .0) return;

        vcg::tri::Allocator<CMeshO>::AddVertices(*m, 1);

        if (uvSpaceFlag) m->vert.back().P() = Point3m(float(tp[0]), float(tp[1]), 0);
        else m->vert.back().P() = f.cP(0) * p[0] + f.cP(1) * p[1] + f.cP(2) * p[2];

        m->vert.back().N() = f.cV(0)->N() * p[0] + f.cV(1)->N() * p[1] + f.cV(2)->N() * p[2];
#if 0
        if (tex)
        {
            QRgb val;
            // Computing normalized texels position
            int xpos = (int)(tex->width() * (float(tp[0]) / texSamplingWidth)) % tex->width();
            int ypos = (int)(tex->height() * (1.0 - float(tp[1]) / texSamplingHeight)) % tex->height();

            if (xpos < 0) xpos += tex->width();
            if (ypos < 0) ypos += tex->height();

            val = tex->pixel(xpos, ypos);
            m->vert.back().C() = Color4b(qRed(val), qGreen(val), qBlue(val), 255);
        }

    }
#endif
    }
};
