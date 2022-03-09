// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;
//Eigen::Vector3d updateTree;

namespace igl
{
namespace opengl
{
namespace glfw
{
    int currSphere = 1;

  void Viewer::Init(const std::string config)
  {

  }
  

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]    Rotate scene
  A,a       Toggle animation (tight draw loop)
  F,f       Toggle face based
  I,i       Toggle invert normals
  L,l       Toggle wireframe
  O,o       Toggle orthographic/perspective projection
  T,t       Toggle filled faces
  [,]       Toggle between cameras
  1,2       Toggle between models
  ;         Toggle vertex labels
  :         Toggle face labels
  [^,v,<,>] Change movement direction)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  //Collision Detection
  void Viewer::CreateLink(int index)
  {
      data_list[index].tree.m_box.center();
      Eigen::MatrixXd V_box(6, 3);
      V_box <<
          0.8, 0, 0.8,
          -0.8, 0, 0.8,
          0, 0.8, 0.8,
          0, -0.8, 0.8,
          0, 0, 0.8,
          0, 0, 2.4;
      data_list[index].add_points(V_box, Eigen::RowVector3d(0, 0, 1));
      Eigen::MatrixXi E_box(3, 2);
      E_box <<
          0, 1,
          2, 3,
          4, 5;
      int x = 1;
      int y = 0;
      int z = 0;
      int count = 0;
      for (unsigned i = 0; i < E_box.rows(); i++) {
          data_list[index].add_edges
          (
              V_box.row(E_box(i, 0)),
              V_box.row(E_box(i, 1)),
              Eigen::RowVector3d(x, y, z)
          );
          z = y;
          y = x;
          x = 0;
      }
  }
 /* void Viewer::Initialize_Tree(int index) {
      data_list.at(index).tree = new igl::AABB<Eigen::MatrixXd, 3>();
      data_list.at(index).tree->init(data_list.at(index).V, data_list.at(index).F);
  }*/


  void Viewer::boundingBox(int index)
  {

      Eigen::AlignedBox <double, 3> box = data_list.at(index).tree.m_box;
      Eigen::MatrixXd V_box(8, 3);
      V_box <<
          box.corner(box.BottomLeftFloor)[0],
          box.corner(box.BottomLeftFloor)[1],
          box.corner(box.BottomLeftFloor)[2],
          box.corner(box.BottomRightFloor)[0],
          box.corner(box.BottomRightFloor)[1],
          box.corner(box.BottomRightFloor)[2],
          box.corner(box.TopLeftFloor)[0],
          box.corner(box.TopLeftFloor)[1],
          box.corner(box.TopLeftFloor)[2],
          box.corner(box.TopRightFloor)[0],
          box.corner(box.TopRightFloor)[1],
          box.corner(box.TopRightFloor)[2],
          box.corner(box.BottomLeftCeil)[0],
          box.corner(box.BottomLeftCeil)[1],
          box.corner(box.BottomLeftCeil)[2],
          box.corner(box.BottomRightCeil)[0],
          box.corner(box.BottomRightCeil)[1],
          box.corner(box.BottomRightCeil)[2],
          box.corner(box.TopLeftCeil)[0],
          box.corner(box.TopLeftCeil)[1],
          box.corner(box.TopLeftCeil)[2],
          box.corner(box.TopRightCeil)[0],
          box.corner(box.TopRightCeil)[1],
          box.corner(box.TopRightCeil)[2];
      // Edges of the bounding box
      Eigen::MatrixXi E_box(12, 2);
      E_box <<
          0, 1,
          1, 3,
          2, 3,
          2, 0,
          4, 5,
          5, 7,
          6, 7,
          6, 4,
          0, 4,
          1, 5,
          2, 6,
          7, 3;

      // Plot the edges of the bounding box
      for (unsigned i = 0; i < E_box.rows(); ++i)
          data().add_edges
          (
              V_box.row(E_box(i, 0)),
              V_box.row(E_box(i, 1)),
              Eigen::RowVector3d(1, 0, 0)
          );
  }
  
  bool Viewer::collision()
  {
      for (int i = 0; i < data_list.size(); i++)
      {
          if (i != selected_data_index)
          {
              //boxesCreated = false;
              if (collision(data_list.at(selected_data_index).tree, selected_data_index, data_list.at(i).tree, i))
                  return true;
          }
      }
      return false;
  }
  bool Viewer::collision(igl::AABB<Eigen::MatrixXd, 3>& A, int indexA, igl::AABB<Eigen::MatrixXd, 3>& B, int indexB)
  {
      Eigen::AlignedBox <double, 3> A_box = A.m_box;
      Eigen::AlignedBox <double, 3> B_box = B.m_box;
      if (didCollide(A_box, indexA, B_box, indexB))
      {
          if (A.is_leaf() & B.is_leaf())
          {
              return true;
          }
          if (A.is_leaf())
          {
              return collision(A, indexA, *B.m_left, indexB) || collision(A, indexA, *B.m_right, indexB);
          }
          if (B.is_leaf())
          {
              return collision(*A.m_left, indexA, B, indexB) || collision(*A.m_right, indexA, B, indexB);
          }
          return collision(*A.m_left, indexA, *B.m_left, indexB) || collision(*A.m_left, indexA, *B.m_right, indexB) || collision(*A.m_right, indexA, *B.m_left, indexB) || collision(*A.m_right, indexA, *B.m_right, indexB);
      }
      return false;
  }
  bool Viewer::didCollide(Eigen::AlignedBox <double, 3>& A_box, int indexA, Eigen::AlignedBox <double, 3>& B_box, int indexB)
  {
      Eigen::Vector3d ARightCol(data_list.at(indexA).MakeTransd()(0, 3), data_list.at(indexA).MakeTransd()(1, 3), data_list.at(indexA).MakeTransd()(2, 3));
      Eigen::Vector3d BRightCol(data_list.at(indexB).MakeTransd()(0, 3), data_list.at(indexB).MakeTransd()(1, 3), data_list.at(indexB).MakeTransd()(2, 3));
      Eigen::Vector3d C0 = A_box.center();
      Eigen::Vector3d C1 = B_box.center();
      Eigen::Matrix3d A = data_list.at(indexA).GetRotation();
      Eigen::Matrix3d B = data_list.at(indexB).GetRotation();
      Eigen::Vector3d D = (B * C1 + BRightCol) - (A * C0 + ARightCol);
      Eigen::Vector3d A0(A(0, 0), A(1, 0), A(2, 0));
      Eigen::Vector3d A1(A(0, 1), A(1, 1), A(2, 1));
      Eigen::Vector3d A2(A(0, 2), A(1, 2), A(2, 2));
      Eigen::Vector3d B0(B(0, 0), B(1, 0), B(2, 0));
      Eigen::Vector3d B1(B(0, 1), B(1, 1), B(2, 1));
      Eigen::Vector3d B2(B(0, 2), B(1, 2), B(2, 2));
      double a0 = A_box.sizes()(0) / 2;
      double a1 = A_box.sizes()(1) / 2;
      double a2 = A_box.sizes()(2) / 2;
      double b0 = B_box.sizes()(0) / 2;
      double b1 = B_box.sizes()(1) / 2;
      double b2 = B_box.sizes()(2) / 2;
      Eigen::Matrix3d C = data_list.at(indexA).GetRotation().transpose() * data_list.at(indexB).GetRotation();
      double R0, R1, R;
      //CASE 1
      R0 = a0;
      R1 = b0 * abs(C(0, 0)) + b1 * abs(C(0, 1)) + b2 * abs(C(0, 2));
      R = (A0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 2
      R0 = a1;
      R1 = b0 * abs(C(1, 0)) + b1 * abs(C(1, 1)) + b2 * abs(C(1, 2));
      R = (A1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 3
      R0 = a2;
      R1 = b0 * abs(C(2, 0)) + b1 * abs(C(2, 1)) + b2 * abs(C(2, 2));
      R = (A2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 4
      R0 = a0 * abs(C(0, 0)) + a1 * abs(C(1, 0)) + a2 * abs(C(2, 0));
      R1 = b0;
      R = (B0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 5
      R0 = a0 * abs(C(0, 1)) + a1 * abs(C(1, 1)) + a2 * abs(C(2, 1));
      R1 = b1;
      R = (B1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 6
      R0 = a0 * abs(C(0, 2)) + a1 * abs(C(1, 2)) + a2 * abs(C(2, 2));
      R1 = b2;
      R = (B2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 7
      R0 = a1 * abs(C(2, 0)) + a2 * abs(C(1, 0));
      R1 = b1 * abs(C(0, 2)) + b2 * abs(C(0, 1));
      R = (C(1, 0) * A2.transpose() * D - C(2, 0) * A1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 8
      R0 = a1 * abs(C(2, 1)) + (a2)*abs(C(1, 1));
      R1 = b0 * abs(C(0, 2)) + b2 * abs(C(0, 0));
      R = (C(1, 1) * A2.transpose() * D - C(2, 1) * A1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 9
      R0 = a1 * abs(C(2, 2)) + a2 * abs(C(1, 2));
      R1 = b0 * abs(C(0, 1)) + b1 * abs(C(0, 0));
      R = (C(1, 2) * A2.transpose() * D - C(2, 2) * A1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 10
      R0 = a0 * abs(C(2, 0)) + a2 * abs(C(0, 0));
      R1 = b1 * abs(C(1, 2)) + b2 * abs(C(1, 1));
      R = (C(2, 0) * A0.transpose() * D - C(0, 0) * A2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 11
      R0 = a0 * abs(C(2, 1)) + a2 * abs(C(0, 1));
      R1 = b0 * abs(C(1, 2)) + b2 * abs(C(1, 0));
      R = (C(2, 1) * A0.transpose() * D - C(0, 1) * A2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 12
      R0 = a0 * abs(C(2, 2)) + a2 * abs(C(0, 2));
      R1 = b0 * abs(C(1, 1)) + b1 * abs(C(1, 0));
      R = (C(2, 2) * A0.transpose() * D - C(0, 2) * A2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 13
      R0 = a0 * abs(C(1, 0)) + a1 * abs(C(0, 0));
      R1 = b1 * abs(C(2, 2)) + b2 * abs(C(2, 1));
      R = (C(0, 0) * A1.transpose() * D - C(1, 0) * A0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 14
      R0 = a0 * abs(C(1, 1)) + a1 * abs(C(0, 1));
      R1 = b0 * abs(C(2, 2)) + b2 * abs(C(2, 0));
      R = (C(0, 1) * A1.transpose() * D - C(1, 1) * A0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      //CASE 15
      R0 = a0 * abs(C(1, 2)) + a1 * abs(C(0, 2));
      R1 = b0 * abs(C(2, 1)) + b1 * abs(C(2, 0));
      R = (C(0, 2) * A1.transpose() * D - C(1, 2) * A0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      return true;
  }

  //Weights Calculation
  Eigen::VectorXd Viewer::createWi(Eigen::Vector4d data)
  {
      Eigen::VectorXd Wi;
      Wi.resize(17);
      double weight1 = data[0];
      double index1 = data[1];
      double weight2 = data[2];
      double index2 = data[3];

      for (double i = 0; i < 17; i++)
      {
          if (i == index1)
              Wi[index1] = weight1;
          else {
              if (i == index2)
                  Wi[index2] = weight2;
              else
                  Wi[i] = 0;
          }
      }
      return Wi;
  }
  void Viewer::CalcWeights()
  {
      int numOfV = data_list.at(1).V.rows();
      Eigen::MatrixXd V = data_list.at(1).V;
      W.resize(numOfV, 17);
      double zCoord;
      double lowBound, upBound;
      double weight1, weight2;
      Eigen::VectorXd weightVector;
      for (int i = 0; i < numOfV; i++)
      {
          zCoord = V.row(i)[2];
          lowBound = (floor(zCoord * 10)) / 10;
          upBound = (ceil(zCoord * 10)) / 10;
          weight1 = abs(zCoord - upBound) * 10;
          weight2 = 1 - weight1;
          W.row(i) = createWi(Eigen::Vector4d(weight1, lowBound * 10 + 8, weight2, upBound * 10 + 8));

      }
  }

  //snake movement
  void Viewer::SetTip()
  {
      Eigen::Vector3d O = (Joints[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
      Eigen::Matrix3d rotation;
      Eigen::Vector3d l;
      spine[0] = O + Joints[0].GetRotation() * Eigen::Vector3d(0, 0, -0.8);
      for (int i = 1; i < jointsNum + 1; i++)
      {
          rotation = Joints[i].GetRotation();
          for (int j = i - 1; j > 0; j--)
          {
              rotation = Joints[j].GetRotation() * rotation;
          }
          l(0) = 0;
          l(1) = 0;
          l(2) = 0.1 * scale;
          O = O + rotation * l;
          spine[i] = O;
      }
  }

  void Viewer::CalcNextPosition()
  {
      for (int i = 0; i < jointsNum + 1; i++)
      {
          vT[i] = spine[i];
      }
      for (size_t i = 0; i < jointsNum; i++)
      {
          vT[i] = vT[i] + (vT[i + 1] - vT[i]) / 6;
      }
      vT[jointsNum] = vT[jointsNum] + destination_position;
  }

  void Viewer::Fabrik()
  {
      chain[jointsNum] = (data_list.at(2).MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
      for (int i = jointsNum - 1; i >= 0; i--)
      {
          Eigen::Vector3d curr = chain[i];
          Eigen::Vector3d next = chain[i + 1];
          double ri = (curr - next).norm();
          double offset = 1.6 / ri;
          chain[i] = (1 - offset) * next + curr * offset;
      }
  }


  void Viewer::moveAlg()
  {
      Eigen::Vector3d currVec;
      Eigen::Vector3d destVec;
      double angle;
      for (int i = 0; i < jointsNum; i++)
      {
          currVec = spine[i] - spine[i + 1];
          destVec = chain[i] - chain[i + 1];
          angle = currVec.normalized().dot(destVec.normalized());
          if (angle > 1)
              angle = 1;
          if (angle < -1)
              angle = -1;
          angle = acos(angle);
          Eigen::Vector3d almostrotVec = currVec.cross(destVec);
          Eigen::Vector4d rotVec(almostrotVec(0), almostrotVec(1), almostrotVec(2), 0);
          Joints[i + 1].MyRotate(((CalcParentsTransJ(i + 1) * Joints[i + 1].MakeTransd()).inverse() * rotVec).head(3), angle / 10);

      }
  }


  Eigen::Matrix4d Viewer::CalcParentsTransJ(int index)
  {
      Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();
      for (int i = index; parentsJoints[i] >= 0; i = parentsJoints[i])
      {

          prevTrans = Joints[parentsJoints[i]].MakeTransd() * prevTrans;
      }
      return prevTrans;
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      int index;
      Eigen::Vector3d center;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
      //data().TranslateInSystem(GetRotation(), Eigen::Vector3d(moveVector/2, 0, 0));
      
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      int joints[17]; //first joint is the head, last is tail
      //int index;
      //Eigen::Vector3d center;


      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
        return false;
      data().set_mesh(V, F);
      /*if (mesh_file_name_string.substr(mesh_file_name_string.length() - 10, mesh_file_name_string.length() - 1) == "snake3.obj") {
          Eigen::Vector3d scale = Eigen::Vector3d(0.5, 0.5, 5);
          //MyScale(scale);
          for (int i = 1; i <= 17; i++) {
              joints[i - 1] = V.rows() - i; //index of the joint in V
          }
      }
      if (mesh_file_name_string.substr(mesh_file_name_string.length() - 10, mesh_file_name_string.length() - 1) == "sphere.obj") {
          data(currSphere).MyTranslate(currSphere * Eigen::Vector3d(1,1,1), true);
          // data(currSphere).Tout.translate(currSphere*Eigen::Vector3d(1,1,1));
          //Eigen::Vector3d min = V.colwise().minCoeff();
          //Eigen::Vector3d max = V.colwise().maxCoeff();
          //center = Eigen::Vector3d((min(0) + max(0)) / 2, (min(1)+max(1))/2, min(2));
          //data().Tin.translate(-center);
          //data().Tout.pretranslate(center);
          currSphere++;
      }
      parents.push_back(-1);
      */
      //data().TranslateInSystem(GetRotation(), Eigen::Vector3d(moveVector/2, 0, 0));
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    // 
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }




  Eigen::Matrix4d Viewer::makeParentsTransd(int index) {
      Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
      if (index == 0)
          return data(0).MakeTransd();
      for (int i = 1; i < index; i++) {
          mat = mat * data(i).MakeTransd();
      }
      return mat;

  }

  Eigen::Matrix3d Viewer::parentsRotationMatrices(int index) {
      Eigen::Matrix3d mat = data(1).Tout.rotation().matrix();
      if (index == 0)
          return Eigen::Matrix3d::Identity();
      else {
          for (int i = 1; i <= index; i++) {
              mat = mat * data(i).Tout.rotation().matrix();
          }
          return mat;
      }
  }

  IGL_INLINE Eigen::Vector3d Viewer::getTip(int index) {
      Eigen::Vector3d ret = data(1).Tout.translation().matrix();
      for (int i = 1; i < index+1; i++) {
          Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
          for (int j = 1; j < i + 1; j++) {
              rot = rot * (data(j).Tout.rotation().matrix());
          }
          ret = ret + (rot * Eigen::Vector3d(0, 0, 1));
      }
      return ret * 1.6;
  }

  IGL_INLINE Eigen::Vector3d Viewer::getHeadLocation() {
      Eigen::Vector3d ret = data(0).Tout.translation().matrix();
      return ret;
  }

  IGL_INLINE Eigen::Vector3d Viewer::getTip() {
      return Eigen::Vector3d::Identity();
  }


  void Viewer::drawAxis(Eigen::Vector3d min, Eigen::Vector3d max) {
      Eigen::MatrixXd V_box(6, 3);
      V_box <<
          (min(0) + max(0)) / 2, max(1) - 1.6, min(2) + 1.6,
          max(0) - 1.6, (min(1)+max(1))/2, max(2),
          min(0) + 1.6, (min(1)+max(1))/2, max(2),
          (min(0) + max(0)) / 2, min(1)+1.6, max(2),
          (max(0) + min(0)) / 2, (min(1)+max(1))/2, max(2),
          0, 0, 2;


      Eigen::MatrixXi E_box(3, 2);
      E_box <<
          0, 3,
          1, 2,
          4, 5;

      data().add_edges(V_box.row(E_box(0, 0)), V_box.row(E_box(0, 1)), Eigen::RowVector3d(1, 0, 0));
      data().add_edges(V_box.row(E_box(1, 0)), V_box.row(E_box(1, 1)), Eigen::RowVector3d(0, 1, 0));
      data().add_edges(V_box.row(E_box(2, 0)), V_box.row(E_box(2, 1)), Eigen::RowVector3d(0, 0, 1));
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {

      if (indx == 0) {
          return  Eigen::Matrix4d::Identity();
      }
      else {
          Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
          for (int i = 1; i < indx; i++) {
              mat = mat * data(i).MakeTransd();
          }
          return mat;
      }
  }


} // end namespace
} // end namespace
}
