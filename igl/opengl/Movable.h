#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>


class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	Eigen::Matrix3d GetRotation() { return Tout.rotation(); }
	void TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt);
	void TranslateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d amt, bool preRotation);
	Eigen::Quaterniond GetRotationQ();
	void RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle);
	Eigen::Vector3d GetCenter();
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void RotateInSystem(Eigen::Vector3d rotAxix, double);
	void MyRotate(const Eigen::Matrix3d &rot);
	void MyScale(Eigen::Vector3d amt);
	void MyRotate(Eigen::Vector3d rotAxis, float angle, bool cond);
	Eigen::Matrix3d GetRotation() const{ return Tout.rotation().matrix(); }

	void SetCenterOfRotation(Eigen::Vector3d amt);
	virtual ~Movable() {}
	Eigen::Affine3d Tout, Tin;
};

