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

		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);

		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);

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
	Eigen::Matrix4d K;
	for (int f = 0; f < data().F.rows(); f++) {
		if (data().F.row(f)(0) == v || data().F.row(f)(1) == v || data().F.row(f)(2) == v) {
			Eigen::Vector3d normal = data().F_normals.row(f).normalized();
			Eigen::Vector3d currV = data().V.row(v);
			double d = abs((normal(0) * currV(0) + normal(1) * currV(1) + normal(2) * currV(2)));
			Eigen::Vector4d p = Eigen::Vector4d(normal(0), normal(1), normal(2), d);
			K += p * p.transpose();
		}
	}
	return K;
}

Eigen::RowVectorXd midpoint(const Eigen::MatrixXd& V, int v0, int v1) {
	Eigen::Matrix4d EdgeMat;
	Eigen::Matrix4d sum = Verror[v0] + Verror[v1];
	EdgeMat << sum(0, 0), sum(0, 1),sum(0, 2), sum(0, 3),
		sum(1, 0), sum(1, 1), sum(1, 2), sum(1, 3),
		sum(2, 0), sum(2, 1), sum(2, 2),sum(2, 3),
		0, 0, 0, 1;
	if (EdgeMat.determinant() != 0) {
		Eigen::Matrix4d inversed = EdgeMat.inverse();
		Eigen::Vector4d res = inversed.row(3);
		return inversed.row(3);
	}
	else {
		Eigen::VectorXd res= (V.row(v0)+V.row(v1))/2;
		return res;
	}
}

double costrewrite(int v0, int v1, Eigen::Vector3d vTag) {
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
		const int max_iter = std::ceil((0.05) * Q.size());
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
		Eigen::RowVectorXd p = midpoint(data().V, v0, v1);
		double cost = costrewrite(v0, v1, p);
		cost_and_midpoint(e, OV, OF, E, EMAP, EF, EI, cost, p, Verror);
		C.row(e) = p;
		Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;
	}
	num_collapsed = 0;
};
