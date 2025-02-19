// Project1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
/*
#include <igl/readOFF.h>
#include <igl/writeOBJ.h>
#include <iostream>


Eigen::MatrixXd V;
Eigen::MatrixXi F;
#define IGL_STATIC_LIBRARY

int main101(int argc, char* argv[])
{
	// Load a mesh in OFF format
	igl::readOFF("E:\\libigl\\002.off", V, F);

	// Print the vertices and faces matrices
	std::cout << "Vertices: " << std::endl << V << std::endl;
	std::cout << "Faces:    " << std::endl << F << std::endl;

	// Save the mesh in OBJ format
	igl::writeOBJ("E:\\libigl\\cube.obj", V, F);
}*/

#include <igl/readOFF.h>
#include <igl/writeOBJ.h>
//#undef IGL_STATIC_LIBRARY
#include <igl/copyleft/cgal/mesh_boolean.h>
//#include <igl/opengl/glfw/Viewer.h>

#include <Eigen/Core>
#include <iostream>


Eigen::MatrixXd VA, VB, VC;
Eigen::VectorXi J, I;
Eigen::MatrixXi FA, FB, FC;
igl::MeshBooleanType boolean_type(
	igl::MESH_BOOLEAN_TYPE_UNION);

const char* MESH_BOOLEAN_TYPE_NAMES[] =
{
  "Union",
  "Intersect",
  "Minus",
  "XOR",
  "Resolve",
};


int main(int argc, char* argv[])
{
	using namespace Eigen;
	using namespace std;
	igl::readOFF("E:\\libigl\\001.off", VA, FA);
	igl::readOFF("E:\\libigl\\002.off", VB, FB);

	//igl::copyleft::cgal::mesh_boolean(VA, FA, VB, FB, igl::MESH_BOOLEAN_TYPE_UNION, VC, FC, J);

	igl::copyleft::cgal::mesh_boolean(VA, FA, VB, FB, igl::MESH_BOOLEAN_TYPE_INTERSECT, VC, FC, J);

	//igl::copyleft::cgal::mesh_boolean(VA, FA, VB, FB, igl::MESH_BOOLEAN_TYPE_MINUS, VC, FC, J);

	//igl::copyleft::cgal::mesh_boolean(VA, FA, VB, FB, igl::MESH_BOOLEAN_TYPE_XOR, VC, FC, J);

	//igl::writeOBJ("E:\\libigl\\booleanXOr.obj", VC, FC);

	igl::writeOBJ("E:\\libigl\\booleanIntersect.obj", VC, FC);
	// Plot the mesh with pseudocolors
	//igl::opengl::glfw::Viewer viewer;

	//// Initialize
	//update(viewer);

	//viewer.data().show_lines = true;
	//viewer.callback_key_down = &key_down;
	//viewer.core().camera_dnear = 3.9;
	//cout <<
	//	"Press '.' to switch to next boolean operation type." << endl <<
	//	"Press ',' to switch to previous boolean operation type." << endl <<
	//	"Press ']' to push near cutting plane away from camera." << endl <<
	//	"Press '[' to pull near cutting plane closer to camera." << endl <<
	//	"Hint: investigate _inside_ the model to see orientation changes." << endl;
	//viewer.launch();
}
