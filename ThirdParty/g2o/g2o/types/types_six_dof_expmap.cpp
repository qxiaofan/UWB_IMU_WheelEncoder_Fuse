// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "types_six_dof_expmap.h"
#include <iostream>

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o {

using namespace std;

CameraParameters
::CameraParameters()
  : focal_length(1.),
    principle_point(Vector2D(0., 0.)),
    baseline(0.5)  {
}

Vector2d project2d(const Vector3d& v)  {
  Vector2d res;
     //   std::cout<<"v(2) = "<<v(2)<<std::endl;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector3d unproject2d(const Vector2d& v)  {
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

Vector2D  CameraParameters::cam_map(const Vector3D & trans_xyz) const {
  Vector2D proj = project2d(trans_xyz);
  Vector2D res;
  res[0] = proj[0]*focal_length + principle_point[0];
  res[1] = proj[1]*focal_length + principle_point[1];
  return res;
}

Vector3D CameraParameters::stereocam_uvu_map(const Vector3D & trans_xyz) const {
  Vector2D uv_left = cam_map(trans_xyz);
  double proj_x_right = (trans_xyz[0]-baseline)/trans_xyz[2];
  double u_right = proj_x_right*focal_length + principle_point[0];
  return Vector3D(uv_left[0],uv_left[1],u_right);
}

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {
}

bool VertexSE3Expmap::read(std::istream& is) {
  Vector7d est;
  for (int i=0; i<7; i++)
    is  >> est[i];
  SE3Quat cam2world;
  cam2world.fromVector(est);
  setEstimate(cam2world.inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(estimate().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  return os.good();
}

EdgeProjectXYZ2UV::EdgeProjectXYZ2UV() : BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ, VertexSE3Expmap>() {
  _cam = 0;
  resizeParameters(1);
  installParameter(_cam, 0);
}

bool EdgeProjectXYZ2UV::read(std::istream& is){
  int paramId;
  is >> paramId;
  setParameterId(0, paramId);

  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeProjectXYZ2UV::write(std::ostream& os) const {
  os << _cam->id() << " ";
  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeProjectXYZ2UV::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3D xyz = vi->estimate();
  Vector3D xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  const CameraParameters * cam = static_cast<const CameraParameters *>(parameter(0));

  Matrix<double,2,3,Eigen::ColMajor> tmp;
  tmp(0,0) = cam->focal_length;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*cam->focal_length;

  tmp(1,0) = 0;
  tmp(1,1) = cam->focal_length;
  tmp(1,2) = -y/z*cam->focal_length;

  _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0,0) =  x*y/z_2 *cam->focal_length;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *cam->focal_length;
  _jacobianOplusXj(0,2) = y/z *cam->focal_length;
  _jacobianOplusXj(0,3) = -1./z *cam->focal_length;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *cam->focal_length;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *cam->focal_length;
  _jacobianOplusXj(1,1) = -x*y/z_2 *cam->focal_length;
  _jacobianOplusXj(1,2) = -x/z *cam->focal_length;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *cam->focal_length;
  _jacobianOplusXj(1,5) = y/z_2 *cam->focal_length;
}


EdgeSE3ProjectEcoLine::EdgeSE3ProjectEcoLine() : BaseBinaryEdge<1, Vector3d, VertexEcoLinePt2, VertexSE3Expmap>() {
}

bool EdgeSE3ProjectEcoLine::read(std::istream& is)
{
    for (int i=0; i<3; i++){
      is >> _measurement[i];
    }
    for (int i=0; i<1; i++){
        is >> information()[0];
      }
    return true;
}

bool EdgeSE3ProjectEcoLine::write(std::ostream& os) const
{

    for (int i=0; i<3; i++){
      os << measurement()[i] << " ";
    }

    os << " " <<  information()(0);
    return os.good();
}


EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectEcoLine::cam_project(const Vector3d & trans_xyz) {
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

void EdgeSE3ProjectEcoLine::linearizeOplus()
{
    //VertexEcoLinePt2, VertexSE3Expmap
    VertexSE3Expmap * v1 = static_cast<VertexSE3Expmap *>(_vertices[1]);
    SE3Quat T(v1->estimate());

    VertexEcoLinePt2* v2 = static_cast<VertexEcoLinePt2*>(_vertices[0]);

    const Eigen::Matrix<double,6,1 > &v2Pos = v2->estimate();
    Vector3d pt1, pt2;
    {
        pt1<<v2Pos[0],v2Pos[1],v2Pos[2];
        pt2<<v2Pos[3],v2Pos[4],v2Pos[5];
    }

    Eigen::Matrix<double, 2,6> jacobianKFPt1;
    Eigen::Matrix<double, 2,6> jacobianKFPt2;
    Eigen::Matrix<double, 2,3> jacobianPt1;
    Eigen::Matrix<double, 2,3> jacobianPt2;
//  cout<<vertex(0)->id()<<"-> "<<vertex(1)->id()<<endl;
//  cout<<"check _jacobianOplusXi "<<_jacobianOplusXi.rows()<< " "<< _jacobianOplusXi.cols()<<endl;
//      cout<<"check _jacobianOplusXi "<<_jacobianOplusXi<<endl;

    {
        Vector3d xyz_trans1 = T.map(pt1);

        double x = xyz_trans1[0];
        double y = xyz_trans1[1];
        double z = xyz_trans1[2];
        double z_2 = z*z;

        Matrix<double,2,3> tmp;
        tmp(0,0) = fx;
        tmp(0,1) = 0;
        tmp(0,2) = -x/z*fx;

        tmp(1,0) = 0;
        tmp(1,1) = fy;
        tmp(1,2) = -y/z*fy;

        jacobianPt1 =  -1./z * tmp * T.rotation().toRotationMatrix();

        jacobianKFPt1(0,0) =  x*y/z_2 *fx;
        jacobianKFPt1(0,1) = -(1+(x*x/z_2)) *fx;
        jacobianKFPt1(0,2) = y/z *fx;
        jacobianKFPt1(0,3) = -1./z *fx;
        jacobianKFPt1(0,4) = 0;
        jacobianKFPt1(0,5) = x/z_2 *fx;

        jacobianKFPt1(1,0) = (1+y*y/z_2) *fy;
        jacobianKFPt1(1,1) = -x*y/z_2 *fy;
        jacobianKFPt1(1,2) = -x/z *fy;
        jacobianKFPt1(1,3) = 0;
        jacobianKFPt1(1,4) = -1./z *fy;
        jacobianKFPt1(1,5) = y/z_2 *fy;
    }

    {
        Vector3d xyz_trans2 = T.map(pt2);

        double x = xyz_trans2[0];
        double y = xyz_trans2[1];
        double z = xyz_trans2[2];
        double z_2 = z*z;

        Matrix<double,2,3> tmp;
        tmp(0,0) = fx;
        tmp(0,1) = 0;
        tmp(0,2) = -x/z*fx;

        tmp(1,0) = 0;
        tmp(1,1) = fy;
        tmp(1,2) = -y/z*fy;

        jacobianPt2 =  -1./z * tmp * T.rotation().toRotationMatrix();

        jacobianKFPt2(0,0) =  x*y/z_2 *fx;
        jacobianKFPt2(0,1) = -(1+(x*x/z_2)) *fx;
        jacobianKFPt2(0,2) = y/z *fx;
        jacobianKFPt2(0,3) = -1./z *fx;
        jacobianKFPt2(0,4) = 0;
        jacobianKFPt2(0,5) = x/z_2 *fx;

        jacobianKFPt2(1,0) = (1+y*y/z_2) *fy;
        jacobianKFPt2(1,1) = -x*y/z_2 *fy;
        jacobianKFPt2(1,2) = -x/z *fy;
        jacobianKFPt2(1,3) = 0;
        jacobianKFPt2(1,4) = -1./z *fy;
        jacobianKFPt2(1,5) = y/z_2 *fy;
    }
//  cout<<jacobianKFPt1<<endl;
//  cout<<jacobianKFPt2<<endl;
//  cout<<"_measurement "<<_measurement<<endl;


    Vector2d uv1 =cam_project(v1->estimate().map(pt1));
    Vector2d uv2 =cam_project(v1->estimate().map(pt2));
//    cout<<"uv1 "<<uv1<<endl;
//    cout<<"uv2 "<<uv2<<endl;
    double a = _measurement[0], b = _measurement[1], c = _measurement[2];
    double dist1 = (a*uv1[0] + b*uv1[1] +c);
    double dist2 = (a*uv2[0] + b*uv2[1] +c);

    Eigen::Matrix<double,1,2> lineAB;
    double norm_ = sqrt(dist1*dist1+dist2*dist2)+1e-10;
//    cout<<"norm_ "<<norm_<<endl;
    double norm_inv = 1 / norm_;
//    cout<<"a*norm_inv "<<a*norm_inv<<endl;
//        cout<<"b*norm_inv "<<b*norm_inv<<endl;
//           cout<<"_jacobianOplusXi "<<_jacobianOplusXi<<endl;

    lineAB<<a*norm_inv,b*norm_inv;


//    cout<<"_jacobianOplusXi "<<_jacobianOplusXi<<endl;
    _jacobianOplusXi.block(0,0,1,3) =-dist1*lineAB*jacobianPt1;
//    cout<<lineAB<<endl;
//    cout<<jacobianPt1<<endl;
//    cout<<dist1<<endl;
//    cout<<"dist2*lineAB*jacobianPt2 "<<dist2<<endl;
//    cout<<"dist2*lineAB*jacobianPt2 "<<lineAB<<endl;
//    cout<<"dist2*lineAB*jacobianPt2 "<<jacobianPt2<<endl;
    //cout<<"dist2*lineAB*jacobianPt2 "<<dist2*lineAB*jacobianPt2<<endl;
    _jacobianOplusXi.block(0,3,1,3) = -dist2*lineAB*jacobianPt2;

//cout<<"end _jacobianOplusXi "<<_jacobianOplusXi<<endl;
//cout<<"end jacobianKFPt1 "<<jacobianKFPt1<<endl;

//cout<<"end jacobianKFPt2 "<<jacobianKFPt2<<endl;

//cout<<"end lineAB "<<lineAB<<endl;
//cout<<"end dist1 "<<dist1<<endl;
//cout<<"end dist2 "<<dist2<<endl;

    _jacobianOplusXj = -1*lineAB*(dist1*jacobianKFPt1+dist2*jacobianKFPt2);
//    cout<<"end _jacobianOplusXj "<<_jacobianOplusXj<<endl;

}


void EdgeSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  Matrix<double,2,3> tmp;
  tmp(0,0) = fx;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*fx;

  tmp(1,0) = 0;
  tmp(1,1) = fy;
  tmp(1,2) = -y/z*fy;

  _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;
}

Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}


Vector3d EdgeStereoSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz, const float &bf) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}

EdgeStereoSE3ProjectXYZ::EdgeStereoSE3ProjectXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeStereoSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<=3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeStereoSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<=3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeStereoSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz);

  const Matrix3d R =  T.rotation().toRotationMatrix();

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  _jacobianOplusXi(0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
  _jacobianOplusXi(0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
  _jacobianOplusXi(0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;

  _jacobianOplusXi(1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
  _jacobianOplusXi(1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
  _jacobianOplusXi(1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;

  _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*R(2,0)/z_2;
  _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*R(2,1)/z_2;
  _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*R(2,2)/z_2;

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;

  _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0)-bf*y/z_2;
  _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1)+bf*x/z_2;
  _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2);
  _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3);
  _jacobianOplusXj(2,4) = 0;
  _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5)-bf/z_2;
}


//Only Pose

bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double invz = 1.0/xyz_trans[2];
  double invz_2 = invz*invz;

  _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
  _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
  _jacobianOplusXi(0,2) = y*invz *fx;
  _jacobianOplusXi(0,3) = -invz *fx;
  _jacobianOplusXi(0,4) = 0;
  _jacobianOplusXi(0,5) = x*invz_2 *fx;

  _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
  _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
  _jacobianOplusXi(1,2) = -x*invz *fy;
  _jacobianOplusXi(1,3) = 0;
  _jacobianOplusXi(1,4) = -invz *fy;
  _jacobianOplusXi(1,5) = y*invz_2 *fy;
}

Vector2d EdgeSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const
{
          Vector2d proj = project2d(trans_xyz);
          Vector2d res;
          res[0] = proj[0]*fx + cx;
          res[1] = proj[1]*fy + cy;
          return res;
}


Vector3d EdgeStereoSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}


bool EdgeStereoSE3ProjectXYZOnlyPose::read(std::istream& is){
  for (int i=0; i<=3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeStereoSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<=3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeStereoSE3ProjectXYZOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double invz = 1.0/xyz_trans[2];
  double invz_2 = invz*invz;

  _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
  _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
  _jacobianOplusXi(0,2) = y*invz *fx;
  _jacobianOplusXi(0,3) = -invz *fx;
  _jacobianOplusXi(0,4) = 0;
  _jacobianOplusXi(0,5) = x*invz_2 *fx;

  _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
  _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
  _jacobianOplusXi(1,2) = -x*invz *fy;
  _jacobianOplusXi(1,3) = 0;
  _jacobianOplusXi(1,4) = -invz *fy;
  _jacobianOplusXi(1,5) = y*invz_2 *fy;

  _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*y*invz_2;
  _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)+bf*x*invz_2;
  _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2);
  _jacobianOplusXi(2,3) = _jacobianOplusXi(0,3);
  _jacobianOplusXi(2,4) = 0;
  _jacobianOplusXi(2,5) = _jacobianOplusXi(0,5)-bf*invz_2;
}

EdgeSE3Expmap::EdgeSE3Expmap() :BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>() {
}

bool EdgeSE3Expmap::read(std::istream& is)  {
  Vector7d meas;
  for (int i=0; i<7; i++)
    is >> meas[i];
  SE3Quat cam2world;
  cam2world.fromVector(meas);
  setMeasurement(cam2world.inverse());
  //TODO: Convert information matrix!!
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(measurement().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeSE3Expmap::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  SE3Quat Ti(vi->estimate());

  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat Tj(vj->estimate());

  const SE3Quat & Tij = _measurement;
  SE3Quat invTij = Tij.inverse();

  SE3Quat invTj_Tij = Tj.inverse()*Tij;
  SE3Quat infTi_invTij = Ti.inverse()*invTij;

  _jacobianOplusXi = invTj_Tij.adj();
  _jacobianOplusXj = -infTi_invTij.adj();
}
Matrix3D Jl(const Vector3D& v3d) {
  double theta = v3d.norm();
  double invtheta = 1. / theta;
  double sint = std::sin(theta);
  double cost = std::cos(theta);
  Vector3D a = v3d * invtheta;

  Matrix3D Jl =
      sint * invtheta * Matrix3D::Identity()
      + ( 1 - sint * invtheta) * a * a.transpose()
      + ( 1 - cost ) * invtheta * g2o::skew(a);
  return Jl;
}

Matrix3D invJl(const Vector3D& v3d) {
  double theta = v3d.norm();
  double thetahalf = theta * 0.5;
  double invtheta = 1. / theta;
  double cothalf = std::tan(M_PI_2 - thetahalf);
  Vector3D a = v3d * invtheta;

  Matrix3D invJl =
      thetahalf * cothalf * Matrix3D::Identity()
      + (1 - thetahalf * cothalf) * a * a.transpose()
      - thetahalf * g2o::skew(a);

  return invJl;
}

Matrix6D AdjTR(const g2o::SE3Quat& pose)
{
  Matrix3D R = pose.rotation().toRotationMatrix();
  Matrix6D res;
  res.block(0,0,3,3) = R;
  res.block(3,3,3,3) = R;
  res.block(0,3,3,3) = g2o::skew(pose.translation())*R;
  res.block(3,0,3,3) = Matrix3D::Zero(3,3);
  return res;
}

Matrix6D AdjTR(const Eigen::Matrix4d& pose)
{
  Matrix3D R = pose.block(0,0,3,3);
  Vector3D t = pose.block(0,3,3,1);
  Matrix6D res;
  res.block(0,0,3,3) = R;
  res.block(3,3,3,3) = R;
  res.block(0,3,3,3) = g2o::skew(t)*R;
  res.block(3,0,3,3) = Matrix3D::Zero(3,3);
  return res;
}

Matrix6D invJJl(const Vector6D& v6d)
{
  //~ ! rho: translation; phi: rotation
  //~ ! vector order: [rot, trans]

  Vector3D rho, phi;
  for(int i = 0; i < 3; i++) {
  phi[i] = v6d[i];
  rho[i] = v6d[i+3];
  }
  double theta = phi.norm();
  Matrix3D Phi = g2o::skew(phi);
  Matrix3D Rho = g2o::skew(rho);
  double sint = std::sin(theta);
  double cost = std::cos(theta);
  double theta2 = theta * theta;
  double theta3 = theta * theta2;
  double theta4 = theta2 * theta2;
  double theta5 = theta4  * theta;
  double invtheta = 1./theta;
  double invtheta3 = 1./theta3;
  double invtheta4 = 1./theta4;
  double invtheta5 = 1./theta5;
  Matrix3D PhiRho = Phi * Rho;
  Matrix3D RhoPhi = Rho * Phi;
  Matrix3D PhiRhoPhi = PhiRho * Phi;
  Matrix3D PhiPhiRho = Phi * PhiRho;
  Matrix3D RhoPhiPhi = RhoPhi * Phi;
  Matrix3D PhiRhoPhiPhi = PhiRhoPhi * Phi;
  Matrix3D PhiPhiRhoPhi = Phi * PhiRhoPhi;

  double temp = (1. - 0.5 * theta2 - cost) * invtheta4;

  Matrix3D Ql =
      0.5 * Rho + (theta - sint) * invtheta3 * (PhiRho + RhoPhi + PhiRhoPhi)
      - temp * (PhiPhiRho + RhoPhiPhi -3. * PhiRhoPhi)
      - 0.5 * (temp - ( 3. * (theta - sint) + theta3 * 0.5) * invtheta5 ) * (PhiRhoPhiPhi + PhiPhiRhoPhi);


  double thetahalf = theta * 0.5;
  double cothalf = std::tan(M_PI_2 - thetahalf);
  Vector3D a = phi * invtheta;
  Matrix3D invJl =
      thetahalf * cothalf * Matrix3D::Identity()
      + (1 - thetahalf * cothalf) * a * a.transpose()
      - thetahalf * g2o::skew(a);

  Matrix6D invJJl = Matrix6D::Zero();
  invJJl.block<3,3>(0,0) = invJl;
  invJJl.block<3,3>(3,0) = - invJl * Ql * invJl;
  invJJl.block<3,3>(3,3) = invJl;
  return invJJl;
}

bool verifyInfo(const Matrix6D& info) {
  bool symmetric = true;
  double th = 0.0001;
  for(int i = 0; i < 6; i++)
  for(int j = 0; j < i; j++)
      symmetric = (std::abs(info(i,j)-info(j,i))<th) && symmetric;
  return symmetric;
}

bool verifyInfo(const Matrix3D& info) {
  double th = 0.0001;
  return (std::abs(info(0,1)-info(1,0))<th &&
      std::abs(info(0,2)-info(2,0))<th &&
      std::abs(info(1,2)-info(2,1))<th);
}

void EdgeSE3ExpmapPrior::computeError()
{
  const g2o::VertexSE3Expmap* v_Tcw = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);

  const g2o::SE3Quat& Tcw = v_Tcw->estimate();
  const g2o::SE3Quat& Tcw_measurement = _measurement;
  g2o::SE3Quat err = Tcw_measurement.inverse() * Tcw;
  _error = err.log();
}

void EdgeSE3ExpmapPrior::linearizeOplus()
{
  const g2o::SE3Quat& Tcw_measurement = _measurement;
  g2o::SE3Quat inv_Tcw_measurement = Tcw_measurement.inverse();

  _jacobianOplusXi = inv_Tcw_measurement.adj();
}

EdgeSE3ExpmapPrior* createEdgeSE3ExpmapPrior(const g2o::SE3Quat& Tcv, g2o::VertexSE3Expmap* v_frame)
{
const g2o::SE3Quat& Tcw = v_frame->estimate();
g2o::SE3Quat Twv = Tcw.inverse() * Tcv;

Eigen::AngleAxisd AngleAxis_wv(Twv.rotation());
Eigen::Vector3d rot_v = AngleAxis_wv.angle() * AngleAxis_wv.axis();
AngleAxis_wv = Eigen::AngleAxisd(rot_v[2], Eigen::Vector3d::UnitZ());
Eigen::Vector3d xyz_wv = Twv.translation(); xyz_wv[2] = 0;

g2o::SE3Quat Twv_measure(AngleAxis_wv.toRotationMatrix(), xyz_wv);
g2o::SE3Quat Tcw_measure = Tcv * Twv_measure.inverse();

// vector order: [rot, trans]
// 	Vector6D v6d = Vector6D::Zero();
// 	v6d[2] = rot_v[2]; v6d[3] = xyz_wv[0]; v6d[4] = xyz_wv[1];
//         Matrix6D invJJl_wv = invJJl(v6d);
Matrix6D invJJl_wv = invJJl(Twv_measure.log());

Matrix6D Info_wheel = Matrix6D::Zero();
float xrot_info = 1e6;
float yrot_info = 1e6;
float z_info = 1.;
// vector order: [rot, trans]
float data[6] = { xrot_info, yrot_info, 1e-4, 1e-4, 1e-4, z_info };
for(int i = 0; i < 6; i++)
    Info_wheel(i,i) = data[i];

Matrix6D info = invJJl_wv.transpose() * Info_wheel * invJJl_wv;
//     assert(verifyInfo(info));

EdgeSE3ExpmapPrior* e = new EdgeSE3ExpmapPrior();
e->setVertex(0, static_cast<g2o::VertexSE3Expmap*>(v_frame));
e->setMeasurement( Tcw_measure );
e->setInformation(info);

return e;
}


} // end namespace
