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
Eigen::Vector3d updateTree;

namespace igl
{
namespace opengl
{
namespace glfw
{
    int currCylinder = 0;

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
      int index;
      Eigen::Vector3d center;


      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }
      if (mesh_file_name_string.substr(mesh_file_name_string.length()-13, mesh_file_name_string.length()-1)=="zcylinder.obj") { //make better
          index = currCylinder;
          data().Tout.translate(Eigen::Vector3d(0, 0, 1.6));
          Eigen::Vector3d min = V.colwise().minCoeff();
          Eigen::Vector3d max = V.colwise().maxCoeff();
          center = Eigen::Vector3d((min(0) + max(0)) / 2, (min(1)+max(1))/2, min(2));
          data().Tin.translate(-center);
          data().Tout.pretranslate(center);
          if (index == 0) {
              data().Tout.translate(Eigen::Vector3d(0, 0, -1.6));
              parents.push_back(0);
          }
          else {
              parents.push_back(index - 1);
          }
          currCylinder++;
          drawAxis(min, max);
      }
      else {
          data().MyTranslate(Eigen::Vector3d(5, 0, 0), true);
          index = -1;
          parents.push_back(-1);
      }

      data().set_mesh(V,F);
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
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }




  Eigen::Matrix4d Viewer::makeParentsTransd(int indexOfLink) {
      Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
      if (indexOfLink == 0)
          return mat;
      for (int i = 1; i < indexOfLink; i++) {
          mat = mat * data(i).MakeTransd();
      }
      return mat;

  }

  Eigen::Matrix3d Viewer::parentsRotationMatrices(int index) {
      Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
      if (index == 0)
          return mat;
      else {
          for (int i = 1; i < index; i++) {
              mat = mat * data(i).Tout.rotation().matrix();
          }
          return mat;
      }
  }

  IGL_INLINE Eigen::Vector3d Viewer::getTip(int index) {
      Eigen::Vector3d ret = Eigen::Vector3d::Zero();
      for (int j = 1; j < index; j++) {
          ret = ret + getRotationVector(j);
      }
      std::cout << ret;
      return ret * 1.6;
  }

  IGL_INLINE Eigen::Vector3d Viewer::getTip() {
      Eigen::Vector3d ret = Eigen::Vector3d::Zero();
      for (int i = 1; i < parents.size(); i++) {
          Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
          for (int j = 1; j < i; j++) {
              rot = rot * (data(i).Tout.rotation().matrix());
          }
          ret = ret + (rot * Eigen::Vector3d(0, 0, 1));
      }
      return ret * 1.6;
  }

  Eigen::Vector3d Viewer::getRotationVector(int index) {
      Eigen::Vector3d vecZ = Eigen::Vector3d(0, 0, 1);
      Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
      for (int i = 1; i < index + 1; i++) {
          mat = mat * data(i).Tout.rotation().matrix();

      }
      return mat * vecZ;

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
	  /*Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();
      //bool cylinder = false;
	  for (int i = indx; i >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
          std::cout << indx;
          //cylinder = true;
	  }
	  return prevTrans;*/
      if (indx == 0) {
          return data(0).MakeTransd();
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
