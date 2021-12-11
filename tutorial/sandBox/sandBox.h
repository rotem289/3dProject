#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include "igl/opengl/ViewerData.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
	void pre_draw();
	void reset();
	Eigen::Matrix4d calcError(int v);
	std::vector<igl::AABB<Eigen::MatrixXd, 3>> trees; //boundingbox

private:
	// Prepare array-based edge data structures and priority queue
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	typedef std::set < std::pair<double, int> > PriorityQueue;
	PriorityQueue Q;
	std::vector<PriorityQueue::iterator > Qit;
	Eigen::MatrixXd C;
	int num_collapsed;


	void Animate();
	//void SetQueue(Eigen::MatrixXd& V, Eigen::MatrixXi& F);


};

