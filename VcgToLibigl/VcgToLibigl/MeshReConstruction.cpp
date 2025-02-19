#include "MeshReConstruction.h"
#include "vcg/complex/allocate.h"
#include <vcg/complex/complex.h>
#include <vcg/space/ray3.h>
#include <vcg/complex/algorithms/intersection.h>
#include <vcg/complex/algorithms/closest.h>
#include <wrap/io_trimesh/export_obj.h>
#include <vcg/complex/algorithms/isotropic_remeshing.h>
#include "poisson_utils.h"
#include <thread>
#include "mymesh.h"
#include "sampling.h"
#include <vcg/complex/algorithms/closest.h>
typedef vcg::tri::FaceTmark<CMeshO> MarkerFace;
#define MESH_SCALE float
class MeshReConstruction::PImpl
{
public:
	PImpl(const CMeshO& mesh) : mesh_(mesh) {}
	~PImpl()
	{}

public:

private:
	CMeshO mesh_;
};

MeshReConstruction::MeshReConstruction(const CMeshO& mesh)
{
	impl_.reset(new PImpl(mesh));
}

MeshReConstruction::~MeshReConstruction()
{
}

#if 0
void MeshReConstruction::FilterPoints(CMeshO& mesh,CMeshO& to_mesh)
{
	vcg::GridStaticPtr<CFaceO, Scalarm> grid;
	vcg::tri::UpdateNormal<CMeshO>::PerVertexNormalized(mesh);
	grid.Set(to_mesh.face.begin(), to_mesh.face.end());
	// 定义射线
	vcg::Ray3<Scalarm> ray;
	Point3m xDir(0.0, 0.0, 1.0);
	vcg::RayTriangleIntersectionFunctor<true> ff;
	std::vector<bool> vertexMarked(mesh.vert.size(), false);
#pragma omp parallel// 使用多线程
	{
		std::vector<bool> localMarked(mesh.vert.size(), false);
		#pragma omp for nowait
		for (int i = 0; i < mesh.vert.size(); ++i)
		{
			ray.SetOrigin(mesh.vert[i].P());
			ray.SetDirection(mesh.vert[i].N()/*.normalized()*/);
			// 射线求交
			CFaceO* hitFace = nullptr;
			Scalarm t;
			Scalarm dist = (std::numeric_limits<Scalarm>::max)();
			hitFace = grid.DoRay(ff, to_mesh, ray, dist, t);
			if (hitFace)
			{
				localMarked[i] = true;
			}
		}

		#pragma omp critical
		{
			for (size_t i = 0; i < localMarked.size(); ++i)
			{
				if (localMarked[i])
				{
					vertexMarked[i] = true;
				}
			}
		}
	}
	// 根据标记设置顶点属性
	for (size_t i = 0; i < vertexMarked.size(); ++i)
	{
		if (vertexMarked[i])
		{
			mesh.vert[i].SetS();
		}
	}
	vcg::tri::UpdateNormal<CMeshO>::PerFace(mesh);
	#pragma omp parallel for
	for (int i = 0; i < mesh.face.size(); ++i)
	{
		bool isDelete = false;
		for (int j = 0; j < 3; j++)
		{
			if (mesh.face[i].V(j)->IsS())
			{
				isDelete = true;
				break;
			}
		}
		if (isDelete)
		{
			mesh.face[i].SetD();
		}
	}
}
#endif
void MeshReConstruction::FilterPoints(CMeshO& mesh, CMeshO& to_mesh)
{
	CMeshO inputMesh;
	vcg::GridStaticPtr<CFaceO, Scalarm> grid;
	grid.Set(to_mesh.face.begin(), to_mesh.face.end());
	// 定义射线
	vcg::Ray3<Scalarm> ray;
	vcg::RayTriangleIntersectionFunctor<false> ff;
	MarkerFace markerFunctor;
	markerFunctor.SetMesh(&to_mesh);
	std::vector<bool> vertexMarked(mesh.vert.size(), false);
	vcg::tri::UpdateNormal<CMeshO>::PerFace(mesh);
	std::vector<bool> localMarked(mesh.vert.size(), false);
	for (auto& f_it : mesh.face)
	{
		vcg::Point3<Scalarm> start_point = (f_it.P(0) + f_it.P(1) + f_it.P(2)) / 3.0f;
		ray.SetOrigin(start_point);
		ray.SetDirection(f_it.N().normalized());
		// 射线求交
		CFaceO* hitFace = nullptr;
		Scalarm t;
		Scalarm dist = (std::numeric_limits<Scalarm>::max)();
		hitFace = grid.DoRay<vcg::RayTriangleIntersectionFunctor<false>, MarkerFace>(ff, markerFunctor, ray, dist, t);
		if (hitFace)
		{
			f_it.SetD();
		}
	}
}

void MeshReConstruction::Remeshing(CMeshO& mesh)
{
	vcg::tri::IsotropicRemeshing<CMeshO>::Params params;
	params.SetTargetLen(0.05);
	params.SetFeatureAngleDeg(10.0);
	params.maxSurfDist = 0.05;
	params.iter = 6;
	params.adapt = true;
	params.selectedOnly = false;
	params.splitFlag = true;
	params.collapseFlag = true;
	params.swapFlag = true;
	params.smoothFlag = true;
	params.projectFlag = true;
	params.surfDistCheck = true;
	//std::cout << mesh_.imark << std::endl;
	CMeshO toProjectCopy = mesh;
	toProjectCopy.face.EnableMark();
	vcg::tri::IsotropicRemeshing<CMeshO>::Do(mesh, toProjectCopy, params);
}

void MeshReConstruction::UpdateMesh(CMeshO& mesh)
{
	vcg::tri::Allocator<CMeshO>::CompactFaceVector(mesh);
	vcg::tri::Allocator<CMeshO>::CompactVertexVector(mesh);
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
	vcg::tri::UpdateNormal<CMeshO>::PerFaceNormalized(mesh);
	vcg::tri::UpdateNormal<CMeshO>::PerVertexFromCurrentFaceNormal(mesh);
}

void MeshReConstruction::FilterFaces(CMeshO& mesh)
{
#if 0 
	vcg::GridStaticPtr<CFaceO, Scalarm> grid;
	grid.Set(mesh.face.begin(), mesh.face.end());
	// 定义射线
	vcg::Ray3<Scalarm> ray;
	Point3m xDir(0.0, 0.0, 1.0);
	vcg::RayTriangleIntersectionFunctor<true> ff;
#pragma omp parallel for // 使用多线程
	for (int i = 0; i < mesh.face.size(); ++i)
	{
		ff.intersectionCount = 0;
		Point3m mid_point = (mesh.face[i].P(0) + mesh.face[i].P(1) + mesh.face[i].P(2))/3.0;
		Point3m face_nor = vcg::NormalizedTriangleNormal(mesh.face[i]).Normalize();
		ray.SetOrigin(mid_point);
		ray.SetDirection(face_nor);
		// 射线求交
		CFaceO* hitFace = nullptr;
		Scalarm t;
		Scalarm dist = (std::numeric_limits<double>::max)();
		hitFace = grid.DoRay(ff, mesh, ray, dist, t);
		int intetp_num = ff.GetIntersectionCount();
		std::cout << intetp_num << std::endl;
		//if (hitFace && t<0.25/*|| ray.Direction() * xDir > 0*/)
		//if(intetp_num%2 !=0)
		if (face_nor * xDir > 0)
		{
			//mesh.face[i].SetD();
			//continue;
			for (int j = 0; j < 3; j++)
			{
				mesh.face[i].V(j)->SetS();
			}
		}
	}
	for (int i = 0; i < mesh.vert.size(); i++)
	{
		if (mesh.vert[i].IsS())
		{
			mesh.vert[i].P().Import(mesh.vert[i].P() + mesh.vert[i].N().Normalize() * 0.5);
		}
	}
#endif
}

void MeshReConstruction::ScaleMesh(CMeshO& mesh, float scale_value = 1.02f)
{
	Eigen::Matrix3f scaleMatrix;
	scaleMatrix.setZero();
	scaleMatrix(0, 0) = scale_value;
	scaleMatrix(1, 1) = scale_value;
	scaleMatrix(2, 2) = scale_value;

	for (auto& v : mesh.vert) {
		Eigen::Vector3f vertex(v.P()[0], v.P()[1], v.P()[2]);
		vertex = scaleMatrix * vertex;  // 应用缩放矩阵
		v.P() = vcg::Point3f(vertex[0], vertex[1], vertex[2]);
	}
}

void MeshReConstruction::PoissonDiskSampling(CMeshO& mesh, CMeshO& result)
{
	SampleParameter par;
	par.SampleNum = 5000;
	
	vcg::tri::SurfaceSampling<CMeshO, BaseSampler>::PoissonDiskParam pp;
	pp.radiusVariance = par.RadiusVariance;
	pp.adaptiveRadiusFlag = false;
	if (par.Radius == 0)
		par.Radius = vcg::tri::SurfaceSampling<CMeshO, BaseSampler>::ComputePoissonDiskRadius(mesh, par.SampleNum);
	else
		par.SampleNum = vcg::tri::SurfaceSampling<CMeshO, BaseSampler>::ComputePoissonSampleNum(mesh, par.Radius);
	CMeshO* presampledMesh = 0;
	CMeshO MontecarloMesh;
	presampledMesh = &MontecarloMesh;
	BaseSampler sampler(presampledMesh);
	sampler.qualitySampling = true;
	vcg::tri::SurfaceSampling<CMeshO, BaseSampler>::Montecarlo(mesh, sampler, par.SampleNum * par.MontecarloRate);
	presampledMesh->bbox = mesh.bbox;
	
	result.updateDataMask();
	BaseSampler mps(&result);
	pp.geodesicDistanceFlag = par.ApproximateGeodesicDistance;
	pp.bestSampleChoiceFlag = par.BestSampleFlag;
	pp.bestSamplePoolSize = par.BestSamplePool;

	vcg::tri::SurfaceSampling<CMeshO, BaseSampler>::PoissonDiskPruning(mps, *presampledMesh, par.Radius, pp);
	vcg::tri::UpdateBounding<CMeshO>::Box(result);
	/*
	parlst.addParam(RichInt("SampleNum", 1000, "Number of samples", "The desired number of samples. The ray of the disk is calculated according to the sampling density."));
	parlst.addParam(RichPercentage("Radius", 0, 0, md.mm()->cm.bbox.Diag(), "Explicit Radius", "If not zero this parameter override the previous parameter to allow exact radius specification"));
	parlst.addParam(RichInt("MontecarloRate", 20, "MonterCarlo OverSampling", "The over-sampling rate that is used to generate the initial Montecarlo samples (e.g. if this parameter is <i>K</i> means that<i>K</i> x <i>poisson sample</i> points will be used). The generated Poisson-disk samples are a subset of these initial Montecarlo samples. Larger this number slows the process but make it a bit more accurate."));
	parlst.addParam(RichBool("SaveMontecarlo", false, "Save Montecarlo", "If true, it will generate an additional Layer with the montecarlo sampling that was pruned to build the poisson distribution."));
	parlst.addParam(RichBool("ApproximateGeodesicDistance", false, "Approximate Geodesic Distance", "If true Poisson Disc distances are computed using an approximate geodesic distance, e.g. an euclidean distance weighted by a function of the difference between the normals of the two points."));
	parlst.addParam(RichBool("Subsample", false, "Base Mesh Subsampling", "If true the original vertices of the base mesh are used as base set of points. In this case the SampleNum should be obviously much smaller than the original vertex number.<br>Note that this option is very useful in the case you want to subsample a dense point cloud."));
	parlst.addParam(RichBool("RefineFlag", false, "Refine Existing Samples", "If true the vertices of the below mesh are used as starting vertices, and they will utterly refined by adding more and more points until possible. "));
	parlst.addParam(RichMesh("RefineMesh", md.mm()->id(), &md, "Samples to be refined", "Used only if the above option is checked. "));
	parlst.addParam(RichBool("BestSampleFlag", true, "Best Sample Heuristic", "If true it will use a simple heuristic for choosing the samples. At a small cost (it can slow a bit the process) it usually improve the maximality of the generated sampling. "));
	parlst.addParam(RichInt("BestSamplePool", 10, "Best Sample Pool Size", "Used only if the Best Sample Flag is true. It control the number of attempt that it makes to get the best sample. It is reasonable that it is smaller than the Montecarlo oversampling factor."));
	parlst.addParam(RichBool("ExactNumFlag", false, "Precise sample number", "If requested it will try to do a dicotomic search for the best poisson disk radius that will generate the requested number of samples with the below specified tolerance. Obviously it will takes much longer."));
	parlst.addParam(RichFloat("ExactNumTolerance", 0.005, "Precise sample number tolerance", "If a precise number of sample is requested, the sample number will be matched with the precision specified here. Precision is specified as a fraction of the sample number. so for example a precision of 0.005 over 1000 samples means that you can get 995 or 1005 samples."));
	parlst.addParam(RichFloat("RadiusVariance", 1, "Radius Variance", "The radius of the disk is allowed to vary between r and r*var. If this parameter is 1 the sampling is the same of the Poisson Disk Sampling"));
	break;
	*/
}

void MeshReConstruction::PoissonReConstruction(CMeshO& mesh, CMeshO& result)
{
	PoissonParam<MESH_SCALE> pp;
	pp.ThreadsVal = std::thread::hardware_concurrency();
	if (pp.ThreadsVal == 0) pp.ThreadsVal = 8;

	bool goodNormal = true, goodColor = true;
	PoissonClean(mesh, pp.ConfidenceFlag, pp.CleanFlag);
	goodNormal &= HasGoodNormal(mesh);

	result.updateDataMask();
	std::string output_name = "D:/data/ConvexHull/result3.obj";
	vcg::tri::io::ExporterOBJ<CMeshO>::Save(mesh, output_name.c_str(), 0);
	

	MeshModelPointStream<MESH_SCALE> meshStream(mesh);
	Box3m bb;
	bb.Add(mesh.Tr, mesh.bbox);
	_Execute<MESH_SCALE, 2, BOUNDARY_NEUMANN, PlyColorAndValueVertex<MESH_SCALE> >(&meshStream, bb, result, pp);
	result.updateBoxAndNormals();
	/*
	parlist.addParam(RichBool("visibleLayer", false, "Merge all visible layers", "Enabling this flag means that all the visible layers will be used for providing the points."));
	parlist.addParam(RichInt("depth", 8, "Reconstruction Depth", "This integer is the maximum depth of the tree that will be used for surface reconstruction. Running at depth d corresponds to solving on a voxel grid whose resolution is no larger than 2^d x 2^d x 2^d. Note that since the reconstructor adapts the octree to the sampling density, the specified reconstruction depth is only an upper bound. The default value for this parameter is 8."));
	parlist.addParam(RichInt("fullDepth", 5, "Adaptive Octree Depth", "This integer specifies the depth beyond depth the octree will be adapted. At coarser depths, the octree will be complete, containing all 2^d x 2^d x 2^d nodes. The default value for this parameter is 5.", true));
	parlist.addParam(RichInt("cgDepth", 0, "Conjugate Gradients Depth", "This integer is the depth up to which a conjugate-gradients solver will be used to solve the linear system. Beyond this depth Gauss-Seidel relaxation will be used. The default value for this parameter is 0.", true));
	parlist.addParam(RichFloat("scale", 1.1, "Scale Factor", "This floating point value specifies the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube. The default value is 1.1.", true));
	parlist.addParam(RichFloat("samplesPerNode", 1.5, "Minimum Number of Samples", "This floating point value specifies the minimum number of sample points that should fall within an octree node as the octree construction is adapted to sampling density. For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction. The default value is 1.5."));
	parlist.addParam(RichFloat("pointWeight", 4, "Interpolation Weight", "This floating point value specifies the importants that interpolation of the point samples is given in the formulation of the screened Poisson equation. The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0. The default value for this parameter is 4."));
	parlist.addParam(RichInt("iters", 8, "Gauss-Seidel Relaxations", "This integer value specifies the number of Gauss-Seidel relaxations to be performed at each level of the hierarchy. The default value for this parameter is 8.", true));
	parlist.addParam(RichBool("confidence", false, "Confidence Flag", "Enabling this flag tells the reconstructor to use the quality as confidence information; this is done by scaling the unit normals with the quality values. When the flag is not enabled, all normals are normalized to have unit-length prior to reconstruction."));
	parlist.addParam(RichBool("preClean", false, "Pre-Clean", "Enabling this flag force a cleaning pre-pass on the data removing all unreferenced vertices or vertices with null normals."));
	parlist.addParam(RichInt("threads", nThreads, "Number Threads", "Maximum number of threads that the reconstruction algorithm can use."));
	*/
}