#include "tutorial\sandBox\sandBox.h"
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
#include <igl/boundary_loop.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/harmonic.h>
#include <igl/dqs.h>
#include <windows.h>
#include <mmsystem.h>


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
	//sound
	//PlaySound(TEXT("127-bpm-hip-hop-beat-loop.wav"), NULL, SND_LOOP | SND_ASYNC);
	
	right = false;
	left = false;
	up = false;
	down = false;
	front = false;
	back = false;
	rotDir = false;
	snakeEye = 0;
	level = 1;
	score = 0;
	finishLevel = false;
	levelWindow = false;
	skinning = false;
	jointsNum = 16;
	spine.resize(jointsNum + 1);
	chain.resize(jointsNum + 1);
	parentsJoints.resize(jointsNum + 1);
	scale = 1;
	vT.resize(17);
	vQuat.resize(17);
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{
		while (nameFileout >> item_name)
		{
			std::cout << "opening " << item_name << std::endl;
			load_mesh_from_file(item_name);
			Eigen::RowVector3d center(0, 0, -0.8);
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			reset();
			data().tree.init(data().V, data().F);
			data().set_visible(false, 1);
			data().SetCenterOfRotation(center.transpose());
			V = data().V;

		}
		if (selected_data_index == 0) {
			data().SetCenterOfRotation(GetCenter().transpose());
		}
		nameFileout.close();

	}
	MyTranslate(Eigen::Vector3d(0,0,-1), true);
	data_list.at(1).MyTranslate(Eigen::Vector3d(0, 0, 0), true);
	data_list.at(0).MyTranslate(Eigen::Vector3d(5, 0, 0), true);


	double z = -0.8 * scale;
	for (int i = 0; i < spine.size(); i++)
	{
		spine.at(i) = Eigen::Vector3d(0, 0, z);
		z = z + 0.1 * scale;

	}


	//Calaulate the weights for each vertex
	CalcWeights();
	data_list.at(1).MyRotate(Eigen::Vector3d(0, 1, 0), 3.14 / 2);

	//Create Joints
	//the first joint that dont have a parent
	Joints.emplace_back();
	Joints.at(0).MyTranslate(spine.at(0), true);

	parentsJoints[0] = -1;
	//the 16 other joint that have parents
	for (int i = 0; i < jointsNum; i++)
	{
		parentsJoints[i + 1] = i;
		Joints.emplace_back();
		Joints.at(i + 1).MyTranslate(spine.at(i + 1), true);


	}


	U = V;
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{
		if (skinning) {
			if (left)
			{
				destination_position = Eigen::Vector3d(-0.07, 0, 0);
			}
			if (right)
			{
				destination_position = Eigen::Vector3d(0.07, 0, 0);
			}
			if (up)
			{
				destination_position = Eigen::Vector3d(0, 0.07, 0);
			}
			if (down)
			{
				destination_position = Eigen::Vector3d(0, -0.07, 0);
			}
			if (front)
			{
				destination_position = Eigen::Vector3d(0, 0, 0.07);
			}
			if (back)
			{
				destination_position = Eigen::Vector3d(0, 0, -0.07);
			}
			//Move The Snake
			CalcNextPosition();
			igl::dqs(V, W, vQuat, vT, U);
			data_list.at(1).set_vertices(U);
			for (size_t i = 0; i < jointsNum + 1; i++)
			{
				spine[i] = vT[i];
			}

		}

		else {
			if (left)
				data().MyTranslate(Eigen::Vector3d(-0.07, 0, 0), true);
			if (right)
				data().MyTranslate(Eigen::Vector3d(0.07, 0, 0), true);
			if (up)
				data().MyTranslate(Eigen::Vector3d(0, 0.07, 0), true);
			if (down)
				data().MyTranslate(Eigen::Vector3d(0, -0.07, 0), true);
			if (front)
				data().MyTranslate(Eigen::Vector3d(0, 0, 0.07), true);
			if (back)
				data().MyTranslate(Eigen::Vector3d(0, 0, -0.07), true);

			if (collision())
			{
				score += 10;
				if (score == 50) {
					finishLevel = true;
					left = false;
					right = false;
					up = false;
					down = false;
					isActive = !isActive;
				}
				//move the ball
				int x = (rand() % 230) - 110;
				double dx = x / 10;
				int y = (rand() % 100) - 50;
				double dy = y / 10;
				int z = (rand() % 210) - 110;
				double dz = z / 10;
				data_list[0].MyTranslate(Eigen::Vector3d(dx, dy, 0), true);

			}
		}
	}
}

void SandBox::SetTexture(int index, std::string path)
{
	Eigen::MatrixXd& V_uv = data_list[index].V_uv;
	Eigen::MatrixXd& V = data_list[index].V;
	Eigen::MatrixXi& F = data_list[index].F;
	Eigen::Vector3d lastRow(F.row(F.rows() / 10 - 1)(0), F.row(F.rows() / 10 - 1)(1), F.row(F.rows() / 10 - 1)(2));
	F.row(F.rows() / 10 - 1)(0) = F.row(F.rows() / 10 - 1)(0);
	F.row(F.rows() / 10 - 1)(1) = F.row(F.rows() / 10 - 1)(0);
	F.row(F.rows() / 10 - 1)(2) = F.row(F.rows() / 10 - 1)(0);
	Eigen::VectorXi bound;
	igl::boundary_loop(F, bound);
	Eigen::MatrixXd bound_uv;
	igl::map_vertices_to_circle(V, bound, bound_uv);
	igl::harmonic(V, F, bound, bound_uv, 1, V_uv);
	V_uv *= 5;
	data_list[index].image_texture(path);
	F.row(F.rows() / 10 - 1)(0) = lastRow(0);
	F.row(F.rows() / 10 - 1)(1) = lastRow(1);
	F.row(F.rows() / 10 - 1)(2) = lastRow(2);

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

