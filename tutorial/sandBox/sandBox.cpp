#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "C:\Users\rotem\Documents\cmake\EngineForAnimationCourse\igl\COLLAPSE_EDGE.H"
#include "Eigen/dense"
#include <functional>
#include <igl/triangle/triangulate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/ViewerData.h"
#include <igl/find.h>
#include "C:/Users/rotem/Documents/cmake/EngineForAnimationCourse/external/glfw/include/GLFW/glfw3.h"
#include "C:\Users\rotem\Documents\cmake\EngineForAnimationCourse\igl\opengl\glfw\renderer.h"
#include<igl/circulation.h>

Eigen::VectorXi EMAP;
Eigen::MatrixXi E, EF, EI;
typedef std::set < std::pair<double, int> > PriorityQueue;
PriorityQueue Q;
std::vector<PriorityQueue::iterator > Qit;
Eigen::MatrixXd C;
std::vector<Eigen::Matrix4d> Verror;
int num_collapsed;

SandBox::SandBox()
{


}

void SandBox::Init(const std::string& config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{
		//double x = 0;
		while (nameFileout >> item_name)
		{
			std::cout << "opening " << item_name << std::endl;
			load_mesh_from_file(item_name);
			//std::cout << data().V;
			moveVector = 1;
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			reset();
			data().tree.init(data().V, data().F);
			//data().setBoundingBox();
			//std::cout << data().V;
			data().set_visible(false, 1);
		}
		nameFileout.close();
	}
	spherePosition = data(0).Tout.translation().matrix();
	MyTranslate(Eigen::Vector3d(0,0,-10), true);
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{



	}
}


Eigen::Matrix4d SandBox::calcError(int v) {
	Eigen::Matrix4d K=Eigen::Matrix4d::Zero();
	std::vector<int> iter = igl::circulation(v, 0, EMAP, EF, EI);
	for(int f=0; f<iter.size();f++){
		Eigen::Vector3d normal = data().F_normals.row(iter[f]).normalized();
		Eigen::Vector3d currV = data().V.row(E(v,0));
		double d = (-1)*(normal(0) * currV(0) + normal(1) * currV(1) + normal(2) * currV(2));
		Eigen::Vector4d p = Eigen::Vector4d(normal(0), normal(1), normal(2), d);
		Eigen::Matrix4d ptag = p * p.transpose();
		K += ptag;
	}
	std::vector<int> iter2 = igl::circulation(v, 1, EMAP, EF, EI);
	for (int f = 0; f < iter2.size(); f++) {
		Eigen::Vector3d normal = data().F_normals.row(iter2[f]).normalized();
		Eigen::Vector3d currV = data().V.row(E(v,1));
		double d = (-1) * (normal(0) * currV(0) + normal(1) * currV(1) + normal(2) * currV(2));
		Eigen::Vector4d p = Eigen::Vector4d(normal(0), normal(1), normal(2), d);
		Eigen::Matrix4d ptag = p * p.transpose();
		K += ptag;
	}
	return K;
}

Eigen::RowVectorXd midpoint(const Eigen::MatrixXd& V, int v0, int v1) {
	Eigen::Matrix4d EdgeMat= Eigen::Matrix4d::Zero();
	Eigen::Matrix4d sum = Verror[v0] + Verror[v1];
	EdgeMat << sum(0, 0), sum(0, 1),sum(0, 2), sum(0, 3),
		sum(1, 0), sum(1, 1), sum(1, 2), sum(1, 3),
		sum(2, 0), sum(2, 1), sum(2, 2),sum(2, 3),
		0, 0, 0, 1;
	if (EdgeMat.determinant() != 0) {
		Eigen::Matrix4d inversed = EdgeMat.inverse();
		Eigen::Vector4d res = inversed*Eigen::Vector4d(0,0,0,1);
		return Eigen::Vector3d(res[0], res[1], res[2]);
	}
	else {
		Eigen::VectorXd res= (V.row(v0)+V.row(v1))/2;
		return res;
	}
}

double costrewrite(int v0, int v1, Eigen::VectorXd vTag) {
	Eigen::Matrix4d sum = Verror[v0] + Verror[v1];
	return (sum(0, 0) * (vTag(0) * vTag(0))
		+ 2 * (sum(0, 1) * (vTag(0) * vTag(1)))
		+ 2 * (sum(0, 2) * (vTag(0) * vTag(2)))
		+ 2 * (sum(0, 3) * vTag(0))
		+ sum(1, 1) * (vTag(1) * vTag(1))
		+ 2 * (sum(1, 2) * (vTag(1) * vTag(2)))
		+ 2 * (sum(1, 3) * vTag(1))
		+ sum(2, 2) * (vTag(2) * vTag(2))
		+ 2 * (sum(2, 3) * vTag(2))
		+ sum(3, 3));
}

IGL_INLINE auto cost_and_midpoint = [](
	const int e,
	const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& /*F*/,
	const Eigen::MatrixXi& E,
	const Eigen::VectorXi& /*EMAP*/,
	const Eigen::MatrixXi& /*EF*/,
	const Eigen::MatrixXi& /*EI*/,
	double& cost,
	Eigen::RowVectorXd& p,
	std::vector<Eigen::Matrix4d>& Verror)-> void
{
	int v0 = E.row(e)(0);
	int v1 = E.row(e)(1);
	Eigen::Vector4d vec0 = Eigen::Vector4d(V.row(v0)(0), V.row(v0)(1), V.row(v0)(2), 1);
	Eigen::Vector4d vec1 = Eigen::Vector4d(V.row(v1)(0), V.row(v1)(1), V.row(v1)(2), 1);
	//calculating vertex according to paper
	p = midpoint(V, v0, v1);
	//calculating cost according to paper
	cost = costrewrite(v0, v1, p);
};


void SandBox::pre_draw()
{
	if (!Q.empty())
	{
		bool something_collapsed = false;
		const int max_iter = std::ceil((0.01) * Q.size());
		for (int j = 0; j < max_iter; j++) {
			if (!igl::collapse_edge(cost_and_midpoint,data().V,data().F,E,EMAP,EF,EI,Q,Qit,C,Verror))
			{
				break;
			}
			something_collapsed = true;
			num_collapsed++;
		}
		if (something_collapsed) {
			for (int v = 0; v < data().V.rows(); v++) {
				Verror[v] = calcError(v);
			}
			data().set_mesh(data().V, data().F);
			data().set_face_based(true);
			data().dirty = 157;
		}
	}

	/*
	1. Compute the Q matrices for all the initial vertices.
	2. Select all valid pairs.
	3. Compute the optimal contraction target v¯ for each valid pair (v1, v2).
	The error v¯T(Q1 + Q2)v¯ of this target vertex becomes the cost of contracting that pair.
	4. Place all the pairs in a heap keyed on cost with the minimum cost pair at the top.
	5. Iteratively remove the pair(v1, v2) of least cost from the heap,
	contract this pair, and update the costs of all valid pairs involving v1.
	*/
};

void SandBox::reset() {
	Eigen::MatrixXi OF = data().F;
	Eigen::MatrixXd OV = data().V;
	igl::edge_flaps(OF, E, EMAP, EF, EI);
	Qit.resize(E.rows());
	C.resize(E.rows(), OV.cols());
	Eigen::VectorXd costs(E.rows());
	Q.clear();
	Verror.resize(OV.rows());
	for (int v = 0; v < OV.rows(); v++) {
		Verror[v] = calcError(v);
	}
	for (int e = 0; e < E.rows(); e++)
	{
		int v0 = E.row(e)(0);
		int v1 = E.row(e)(1);
		Eigen::RowVectorXd p(1, 3);
		p = midpoint(data().V, v0, v1);
		double cost = costrewrite(v0, v1, p);
		cost_and_midpoint(e, OV, OF, E, EMAP, EF, EI, cost, p, Verror);
		C.row(e) = p;
		Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;
	}
	num_collapsed = 0;
};

