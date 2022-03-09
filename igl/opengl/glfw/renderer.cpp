#include "RENDERER.H"
#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include "igl/opengl/Movable.h"
//#include <Eigen/Dense>

Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	//highdpi = 1;

	xold = 0;
	yold = 0;
}

IGL_INLINE void Renderer::draw(GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;

	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);

	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window, width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}
	int coreIndx = 1;
}
void Renderer::DrawMenu() {
	if (menu)
	{
		menu->pre_draw();
		menu->callback_draw_viewer_menu();
	}
	for (auto& core : core_list)
	{
		int indx = 0;
		for (auto& mesh : scn->data_list)
		{
			
			if (mesh.is_visible & core.id)
			{// for kinematic chain change scn->MakeTrans to parent matrix
				
				core.draw(scn->MakeTransScale()*scn->CalcParentsTrans(indx).cast<float>(),mesh);
			}
			indx++;
		}

		
	}
	if (menu)
	{
		menu->post_draw();
		
	}

}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer,int coresNum, igl::opengl::glfw::imgui::ImGuiMenu* _menu, Display *_display)
{
	moveCamera = false;
	scn = viewer;
	display = _display;
	doubleVariable = 0;
	core().init(); 
	menu = _menu;
	core().align_camera_center(scn->data().V, scn->data().F);

	if (coresNum > 1)
	{	
		int width = core().viewport[2];
		int height = core().viewport[3];
		
		core().viewport = Eigen::Vector4f(0, 0, width/4, height);
		left_view = core_list[0].id;
		right_view = append_core(Eigen::Vector4f(width/4, 0, width*3/4, height));
		core_index(right_view - 1);
		for (size_t i = 0; i < scn->data_list.size(); i++)
		{
			core().toggle(scn->data_list[i].show_faces);
			core().toggle(scn->data_list[i].show_lines);
			core().toggle(scn->data_list[i].show_texture );
			core().toggle(scn->data_list[i].show_overlay);
			core().toggle(scn->data_list[i].show_overlay_depth);

		}
		//Eigen::Vector3d v = -scn->GetCameraPosition();
		//TranslateCamera(v.cast<float>());

		core_index(left_view - 1);
	}

	if (menu)
	{
		menu->callback_draw_viewer_menu = [&]()
		{
			// Draw parent menu content
			menu->draw_viewer_menu(scn,core_list);


		};
	}
}

void Renderer::IKcheck() {
	if ((spherePosition() - bottom()).norm() > (scn->data_list.size() - 1) * 1.6) {
		IKrun = false;
	}
}

void Renderer::IKswitch() {
	if ((spherePosition() - bottom()).norm() < (scn->data_list.size()-1)*1.6) {
		//std::cout << "sphere " << spherePosition() << " bottom " << bottom() << " norm " << (spherePosition() - bottom()).norm() << '\n';
		IKrun = !IKrun;
	}
	else
		std::cout << "Cannot Reach! \n";
}

Eigen::Vector3d Renderer::bottom() {
	return scn->data(1).Tout.translation().matrix();
}

Eigen::Vector3d Renderer::spherePosition() {
	return scn->data(0).Tout.translation().matrix();
}

void Renderer::IKfabrik()
{

	std::vector<Eigen::Vector3d> pi, tips;
	pi.resize(scn->data_list.size());
	tips.resize(scn->data_list.size());
	pi[0] = bottom();
	tips[0] = bottom();
	//std::cout << "bottom: " << tips[0].transpose()<<'\n';
	for (int i = 1; i < scn->data_list.size(); i++) {
		pi[i] = scn->getTip(i);
		tips[i] = pi[i];
		//std::cout << "tip: " << tips[i].transpose() << '\n';
	}
	Eigen::Vector3d t = spherePosition();
	double lambda;
	double r;
	Eigen::Vector3d b = pi[0];
	
	// STAGE 1: FORWARD REACHING           
	pi[pi.size() - 1] = t; 
	for (int i = pi.size() - 2; i >= 0; i--) {
		r = (pi[i + 1] - pi[i]).norm();   
		lambda = 1.6 / r;
		pi[i] = (1 - lambda) * pi[i + 1] + lambda * pi[i]; 

	}
	
	//STAGE 2: BACKWARD REACHING
	pi[0] = b;  
	for (int i = 0; i < pi.size() - 1; i++) {
		r = (pi[i + 1] - pi[i]).norm();   
		lambda = 1.6 / r;
		pi[i + 1] = (1 - lambda) * pi[i] + lambda * pi[i + 1];
	}

	for (int i = 0; i < scn->data_list.size() - 1; i++) {
		Eigen::Vector4d prevPos, newPos;
		prevPos << (tips[i + 1] - tips[i]), 1;
		newPos << (pi[i + 1] - pi[i]), 1;
		double A = prevPos.normalized().dot(newPos.normalized());
		if (A > 1)
			A = 1;
		if (A < -1)
			A = -1;
		double alpha = acos(A);

		scn->data_list[i + 1].MyRotate(((scn->CalcParentsTrans(i + 1) * scn->data_list[i + 1].MakeTransd()).inverse() * prevPos.cross3(newPos)).head(3), alpha / 10);
		

		for (int i = 1; i < scn->data_list.size(); i++) {
			Eigen::Vector3d euAngles = scn->data_list[i].GetRotation().eulerAngles(2, 0, 2);
			scn->data_list[i].MyRotate(Eigen::Vector3d(0, 0, 1), -euAngles(2));
			if (i < scn->data_list.size() - 1)
				scn->data_list[i + 1].RotateInSystem(Eigen::Vector3d(0, 0, 1), euAngles(2));
		}

	double distance = distanceFromSphere();
	if (distance < 0.1) {
		IKrun = false;
		std::cout << "distance is:  " << distance << '\n';
	}
}
}

void Renderer::printRotation() {
	if (!scn->isPicked && scn->selected_data_index > 0) {
		std::cout << "Rotation matrix: " << (scn->makeParentsTransd(scn->selected_data_index) * scn->data().MakeTransd()).block<3, 3>(0, 0) << '\n';
	}
	else
		std::cout << "Rotation matrix of scene: " << (scn->MakeTransd()).block<3, 3>(0, 0) << '\n';
}

void Renderer::printTip() {
	std::cout<<"The Arm Tip: "<< (scn->getTip()).transpose()<<'\n';
}

void Renderer::printSphere() {
	std::cout << "The sphere is at: " << spherePosition().transpose() << '\n';
}

IGL_INLINE void Renderer::RotateYAxis(std::string direction) {
	if (scn->isPicked) {
		//scn rotation
		if (direction.compare("right")) { scn->MyRotate(Eigen::Vector3d(0, 1, 0), 0.1); }
		else { scn->MyRotate(Eigen::Vector3d(0, 1, 0), -0.1); }
	}
	else {
		Eigen::Matrix3d rotate = scn->data().Tout.rotation().matrix();
		if (direction.compare("right")) {
			scn->data().Tout.rotate(rotate.inverse());
			scn->data().MyRotate(Eigen::Vector3d(0, 1, 0), 0.1);
			scn->data().Tout.rotate(rotate);

		}
		else {
			scn->data().Tout.rotate(rotate.inverse());
			scn->data().MyRotate(Eigen::Vector3d(0, 1, 0), -0.1);
			scn->data().Tout.rotate(rotate);

		}

	}

}

IGL_INLINE void Renderer::RotateXAxis(std::string direction) {
	if (scn->isPicked) {
		//scn rotation
		if (direction.compare("up")) { scn->MyRotate(Eigen::Vector3d(1, 0, 0), 20 / 180.0f); }
		else { scn->MyRotate(Eigen::Vector3d(1, 0, 0), -20 / 180.0f); }
	}
	else {
		Eigen::Matrix3d rotate = scn->data().Tout.rotation().matrix();
		if (direction.compare("up")) {
			scn->data().MyRotate(Eigen::Vector3d(1, 0, 0), 0.1);
		}
		else {
			scn->data().MyRotate(Eigen::Vector3d(1, 0, 0), -0.1);
		}

	}

}

IGL_INLINE void Renderer::IKCyclic() {
	Eigen::Vector3d D = spherePosition();

	for (int i = (scn->data_list.size()-1); IKrun && i > 0; i--)
	{
		Eigen::Vector3d e = scn->getTip();// get the tip of the last cylinder
		Eigen::Vector3d er, rd, r;
		if (i == 1) {
			r = bottom();
		}
		else {
			r = scn->getTip(i-1);
		}


		er = (e - r).normalized();
		rd = (D - r).normalized();

		double distance = distanceFromSphere();

		if (distance < 0.1) {
			std::cout << "distance is:  " << distance << '\n';
			IKrun = false;
			return;
		}
		double A= er.dot(rd);
		if (A >= 1)
			A = 1;
		if (A <= -1)
			A = -1;
		double alpha = acos(A);
		//std::cout << scn->parentsRotationMatrices(1);
		Eigen::Vector3d axis = scn->parentsRotationMatrices(i).inverse() * er.cross(rd).normalized();


		scn->data_list[i].MyRotate(axis.normalized(), alpha/10);

		IKcheck();
	}
}


/*for (int i = 1; i < scn->data_list.size(); i++) {
	Eigen::Vector3d eAngles = scn->data_list[i].GetRotation().eulerAngles(2, 0, 2);
	scn->data_list[i].MyRotate(Eigen::Vector3d(0, 0, 1), -eAngles(2));
	if (i < scn->data_list.size() - 1)
		scn->data_list[i + 1].RotateInSystem(Eigen::Vector3d(0, 0, 1), eAngles(2));
}*/

double Renderer::distanceFromSphere() {
	Eigen::Vector3d e = spherePosition() - (scn->getTip());// get the tip of the last cylinder
	return e.norm();
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::MouseProcessing(int button)
{
	if (scn->isPicked) //object
	{
		if (button == 1)
		{
			float near = core().camera_dnear, far = core().camera_dfar, angle = core().camera_view_angle;
			//float z = far + depth * (near - far);
			
			Eigen::Matrix4f tmpM = core().proj;
			double xToMove = -(double)xrel / core().viewport[3] * (z+2*near) * (far) / (far + 2*near) * 2.0 * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			double yToMove = (double)yrel / core().viewport[3] *(z+2*near) * (far ) / (far+ 2*near) * 2.0 * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			if (scn->selected_data_index != 0) {
				scn->data(1).TranslateInSystem(scn->GetRotation(), Eigen::Vector3d(xToMove, 0, 0));
				scn->data(1).TranslateInSystem(scn->GetRotation(), Eigen::Vector3d(0, yToMove, 0));
			}
			else
			{
				scn->data().TranslateInSystem(scn->GetRotation(), Eigen::Vector3d(xToMove, 0, 0));
				scn->data().TranslateInSystem(scn->GetRotation(), Eigen::Vector3d(0, yToMove, 0));
			}
			scn->WhenTranslate();
			moveCamera = true;
		}
		else
		{
			scn->data().RotateInSystem(Eigen::Vector3d(1, 0, 0), -yrel / 100.0);
			scn->data().RotateInSystem(Eigen::Vector3d(0, 1, 0), -xrel / 100.0);

		}
	}
	else //scene
	{
		if (button == 1)
		{
			float near = core().camera_dnear, far = core().camera_dfar, angle = core().camera_view_angle;
			float z = far + 0.5f * (near - far);
			

			double xToMove = -(double)xrel / core().viewport[3] * far / z * near * 2.0f * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			double yToMove = (double)yrel / core().viewport[3] * far / z * near * 2.0f * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			scn->MyTranslate(Eigen::Vector3d(xToMove, 0, 0), true);
			scn->MyTranslate(Eigen::Vector3d(0, yToMove, 0), true);
			moveCamera = true;
		}
		else
		{
			scn->RotateInSystem(Eigen::Vector3d(1, 0, 0), yrel / 100.0);
			scn->RotateInSystem(Eigen::Vector3d(0, 1, 0), xrel / 100.0);

		}
	}
}

void Renderer::moveObject() {
	scn->data(0).MyTranslate(Eigen::Vector3d(0,0,0.05), false);
}

void Renderer::rotateObject(int x)
{
	switch (x) {
	case 0:  //up
		scn->data(0).MyRotate(Eigen::Vector3d(1, 0, 0), 0.1);
		break;
	case 1: //down
		scn->data(0).MyRotate(Eigen::Vector3d(1, 0, 0), -0.1);
		break;
	case 2: //left
		scn->data(0).MyRotate(Eigen::Vector3d(0, 1, 0), 0.1);
		break;
	case 3: //right
		scn->data(0).MyRotate(Eigen::Vector3d(0, 1, 0), -0.1);
		break;
	case 4:
		break;
	default:
		break;
	}

}

void Renderer::TranslateCamera(Eigen::Vector3f amt)
{
	core().camera_translation += amt;
}

void Renderer::RotateCamera(float amtX, float amtY)
{
	core().camera_eye = core().camera_eye + Eigen::Vector3f(0,amtY,0);
	Eigen::Matrix3f Mat;
		Mat << cos(amtY),0,sin(amtY),  0, 1, 0 ,  -sin(amtY), 0, cos(amtY) ;
	core().camera_eye = Mat* core().camera_eye;
	
}

void Renderer::cameraSnakeEye()
{
	if (scn->snakeEye) {
			core().camera_translation = headLocation().cast<float>(); //head location
			core().camera_eye << CameraEye(); //rotation
			core().camera_up << CameraUp(); //how the snake is rotated around itself
		}
	else
	{
		core().camera_eye << 0, 0, 20;
		core().camera_center << 0, 0, 0;
		core().camera_up << 0, 1, 0;
	}
}


Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

double Renderer::Picking(double newx, double newy)
{
		int fid;
		//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
		Eigen::Vector3f bc;
		double x = newx;
		double y = core().viewport(3) - newy;
		
		Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
		
		igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
		//std::cout << "view matrix\n" << view << std::endl;
		view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
				* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTransScale()* scn->CalcParentsTrans(scn->selected_data_index).cast<float>() * scn->data().MakeTransScale();
		bool picked = igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
			core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc);
		scn->isPicked = scn->isPicked | picked;
		if (picked)
		{
			Eigen::Vector3i face = scn->data().F.row(fid);
			Eigen::Matrix3d vertices ;
			Eigen::Vector4f p,pp ;

			vertices.col(0) = scn->data().V.row(face(0));
			vertices.col(1) =  scn->data().V.row(face(1));
			vertices.col(2) = scn->data().V.row(face(2));
		
			p <<  vertices.cast<float>() * bc ,1;
			p = view * p;
			//std::cout << scn->data().V.row(face(0)) << std::endl;
			pp = core().proj * p;
			//glReadPixels(x,  y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
			z = pp(2);
			return p(2);
			
		}
		return 0;
}

IGL_INLINE void Renderer::resize(GLFWwindow* window,int w, int h)
	{
		if (window) {
			glfwSetWindowSize(window, w / highdpi, h / highdpi);
		}
		post_resize(window,w, h);
	}

	IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
	{
		if (core_list.size() == 1)
		{
	
			core().viewport = Eigen::Vector4f(0, 0, w, h);
		}
		else
		{
			// It is up to the user to define the behavior of the post_resize() function
			// when there are multiple viewports (through the `callback_post_resize` callback)
			core(left_view).viewport = Eigen::Vector4f(0, 0, w / 4, h);
			core(right_view).viewport = Eigen::Vector4f(w / 4, 0, w - (w / 4), h);

		}
		//for (unsigned int i = 0; i < plugins.size(); ++i)
		//{
		//	plugins[i]->post_resize(w, h);
		//}
		if (callback_post_resize)
		{
			callback_post_resize(window, w, h);
		}
	}

	IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE bool Renderer::erase_core(const size_t index)
	{
		assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
		//assert(data_list.size() >= 1);
		if (core_list.size() == 1)
		{
			// Cannot remove last viewport
			return false;
		}
		core_list[index].shut(); // does nothing
		core_list.erase(core_list.begin() + index);
		if (selected_core_index >= index && selected_core_index > 0)
		{
			selected_core_index--;
		}
		return true;
	}

	IGL_INLINE size_t Renderer::core_index(const int id) const {
		for (size_t i = 0; i < core_list.size(); ++i)
		{
			if (core_list[i].id == id)
				return i;
		}
		return 0;
	}

	int Renderer::collision() {
		//std::cout << "data size:" << scn->data_list.size();
		for (int i = 1; i < scn->data_list.size(); i++) {
			if (boxCollide(&scn->data(0).tree, &scn->data(i).tree,i)) {
				std::cout << i;
				scn->data(i).clear();
				return i;
			}
		}
		return -1;
	}

	IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
	{
		core_list.push_back(core()); // copies the previous active core and only changes the viewport
		core_list.back().viewport = viewport;
		core_list.back().id = next_core_id;
		next_core_id <<= 1;
		if (!append_empty)
		{
			for (auto& data : scn->data_list)
			{
				data.set_visible(true, core_list.back().id);
				//data.copy_options(core(), core_list.back());
			}

		}
		selected_core_index = core_list.size() - 1;
		return core_list.back().id;
	}

	bool Renderer::boxCollide(igl::AABB<Eigen::MatrixXd, 3> *tree1, igl::AABB<Eigen::MatrixXd, 3> *tree2,int i) {

		Eigen::Vector3d center1 = tree1->m_box.center();
		Eigen::Vector3d center2 = tree2->m_box.center();

		Eigen::Matrix3d A = scn->data(0).Tout *Eigen::Matrix3d::Identity();
		Eigen::Matrix3d B = scn->data(i).Tout * Eigen::Matrix3d::Identity();
		Eigen::Matrix3d C = (A.transpose() * B);
		Eigen::Vector3d C0 = (scn->data(0).MakeTransScaled() * Eigen::Vector4d(center1[0], center1[1], center1[2], 1)).head(3);
		Eigen::Vector3d C1 = (scn->data(i).MakeTransScaled() * Eigen::Vector4d(center2[0], center2[1], center2[2], 1)).head(3);

		Eigen::Vector3d D = C1 - C0;
		double ax = tree1->m_box.sizes().cast<double>()[0] / 2;
		double ay = tree1->m_box.sizes().cast<double>()[1] / 2;
		double az = tree1->m_box.sizes().cast<double>()[2] / 2;

		double bx = tree2->m_box.sizes().cast<double>()[0] / 2;
		double by = tree2->m_box.sizes().cast<double>()[1] / 2;
		double bz = tree2->m_box.sizes().cast<double>()[2] / 2;

		bool separate = overlap(A.col(0), A.col(1), A.col(2), ax, ay, az, B.col(0), B.col(1), B.col(2), bx, by, bz, C, D);
		if (!separate) {
			if (tree1->is_leaf()) {
				if (tree2->is_leaf()) { //both leaves
					//direction = 4;
					//finalBox(&tree1->m_box, 0);
					//finalBox(&tree2->m_box, i);
					return true;
				}
				else //only 1 is leaf
					return boxCollide(tree1, tree2->m_right,i) || boxCollide(tree1, tree2->m_left,i);
			}
			else if (tree2->is_leaf()) //only 2 is leaf
				return boxCollide(tree1->m_right, tree2,i) || boxCollide(tree1->m_left, tree2,i);
			else //no leaves
				return boxCollide(tree1->m_right, tree2->m_right,i) || boxCollide(tree1->m_right, tree2->m_left,i) ||
				boxCollide(tree1->m_left, tree2->m_right,i) || boxCollide(tree1->m_left, tree2->m_left,i);
		}
		else
			return false;
	}

	void Renderer::printHead() {
		std::cout << headLocation() <<'\n';
	}

	Eigen::Vector3d Renderer::headLocation() {
		return scn->data(1).Tout.translation().matrix().transpose()*(0,0,0.8);
	}

	Eigen::Vector3f Renderer::CameraUp() {
		return scn->data(0).Tout.rotation().matrix().cast<float>() * Eigen::Vector3f(1, 0, 0);
	}

	Eigen::Vector3f Renderer::CameraEye() {
		return scn->data(0).Tout.rotation().matrix().cast<float>() * Eigen::Vector3f(0, 0, 1);
	}


	IGL_INLINE void Renderer::finalBox(Eigen::AlignedBox<double,3> *box, int objIndex)
	{
		// Find the bounding box
		Eigen::Vector3d m = box->corner(box->BottomLeftFloor);
		Eigen::Vector3d M = box->corner(box->TopRightCeil);

		// Corners of the bounding box
		Eigen::MatrixXd V_box(8, 3);
		V_box <<
			m(0), m(1), m(2),
			M(0), m(1), m(2),
			M(0), M(1), m(2),
			m(0), M(1), m(2),
			m(0), m(1), M(2),
			M(0), m(1), M(2),
			M(0), M(1), M(2),
			m(0), M(1), M(2);

		// Edges of the bounding box
		Eigen::MatrixXi E_box(12, 2);
		E_box <<
			0, 1,
			1, 2,
			2, 3,
			3, 0,
			4, 5,
			5, 6,
			6, 7,
			7, 4,
			0, 4,
			1, 5,
			2, 6,
			7, 3;

		// Plot the corners of the bounding box as points
		scn->data(objIndex).add_points(V_box, Eigen::RowVector3d(1, 0, 0));

		// Plot the edges of the bounding box
		for (unsigned i = 0; i < E_box.rows(); ++i)
			scn->data(objIndex).add_edges
			(
				V_box.row(E_box(i, 0)),
				V_box.row(E_box(i, 1)),
				Eigen::RowVector3d(0, 0, 0)
			);
	}


	bool Renderer::overlap(Eigen::RowVector3d A0, Eigen::RowVector3d A1, Eigen::RowVector3d A2, double a0, double a1,
		double a2, Eigen::RowVector3d B0, Eigen::RowVector3d B1, Eigen::RowVector3d B2, double b0,
		double b1, double b2, Eigen::Matrix3d C, Eigen::Vector3d D)
	{
		double c00 = C(0,0), c01 = C(0,1), c02 = C(0, 2), 
			   c10 = C(1, 0), c11 = C(1, 1), c12 = C(1, 2), 
			   c20 = C(2, 0), c21 = C(2, 1), c22 = C(2, 2);
		return
			a0 + b0 * abs(c00) + b1 * abs(c01) + b2 * abs(c02) < abs(A0.dot(D)) ||
			a1 + b0 * abs(c10) + b1 * abs(c11) + b2 * abs(c12) < abs(A1.dot(D)) ||
			a2 + b0 * abs(c20) + b1 * abs(c21) + b2 * abs(c22) < abs(A2.dot(D)) ||
			b0 + a0 * abs(c00) + a1 * abs(c10) + a2 * abs(c20) < abs(B0.dot(D)) ||
			b1 + a0 * abs(c01) + a1 * abs(c11) + a2 * abs(c21) < abs(B1.dot(D)) ||
			b2 + a0 * abs(c02) + a1 * abs(c12) + a2 * abs(c22) < abs(B2.dot(D)) ||

			a1 * abs(c20) + a2 * abs(c10) + b1 * abs(c02) + b2 * abs(c01) < abs(c10 * A2.dot(D) - c20 * A1.dot(D)) ||
			a1 * abs(c21) + a2 * abs(c11) + b0 * abs(c02) + b2 * abs(c00) < abs(c11 * A2.dot(D) - c21 * A1.dot(D)) ||
			a1 * abs(c22) + a2 * abs(c12) + b0 * abs(c01) + b1 * abs(c00) < abs(c12 * A2.dot(D) - c22 * A1.dot(D)) ||

			a0 * abs(c20) + a2 * abs(c00) + b1 * abs(c12) + b2 * abs(c11) < abs(c20 * A0.dot(D) - c00 * A2.dot(D)) ||
			a0 * abs(c21) + a2 * abs(c01) + b0 * abs(c12) + b2 * abs(c10) < abs(c21 * A0.dot(D) - c01 * A2.dot(D)) ||
			a0 * abs(c22) + a2 * abs(c02) + b0 * abs(c11) + b1 * abs(c10) < abs(c22 * A0.dot(D) - c02 * A2.dot(D)) ||

			a0 * abs(c10) + a1 * abs(c00) + b1 * abs(c22) + b2 * abs(c21) < abs(c00 * A1.dot(D) - c10 * A0.dot(D)) ||
			a0 * abs(c11) + a2 * abs(c01) + b0 * abs(c22) + b2 * abs(c20) < abs(c01 * A1.dot(D) - c11 * A0.dot(D)) ||
			a0 * abs(c12) + a2 * abs(c02) + b0 * abs(c21) + b1 * abs(c20) < abs(c02 * A1.dot(D) - c12 * A0.dot(D));

	}
	//IGL_INLINE void Viewer::select_hovered_core()
	//{
	//	int width_window, height_window = 800;
	//   glfwGetFramebufferSize(window, &width_window, &height_window);
	//	for (int i = 0; i < core_list.size(); i++)
	//	{
	//		Eigen::Vector4f viewport = core_list[i].viewport;

	//		if ((current_mouse_x > viewport[0]) &&
	//			(current_mouse_x < viewport[0] + viewport[2]) &&
	//			((height_window - current_mouse_y) > viewport[1]) &&
	//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
	//		{
	//			selected_core_index = i;
	//			break;
	//		}
	//	}
	//}