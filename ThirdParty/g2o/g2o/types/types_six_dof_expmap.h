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

// Modified by Raúl Mur Artal (2014)
// Added EdgeSE3ProjectXYZ (project using focal_length in x,y directions)
// Modified by Raúl Mur Artal (2016)
// Added EdgeStereoSE3ProjectXYZ (project using focal_length in x,y directions)
// Added EdgeSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)
// Added EdgeStereoSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)

#ifndef G2O_SIX_DOF_TYPES_EXPMAP
#define G2O_SIX_DOF_TYPES_EXPMAP

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "se3_ops.h"
#include "se3quat.h"
#include "types_sba.h"
#include <Eigen/Geometry>
#include"../core/eigen_types.h"
#include <iostream>
#include <opencv2/core/core.hpp>

namespace g2o {
namespace types_six_dof_expmap {
void init();
}

using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;

class CameraParameters : public g2o::Parameter
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CameraParameters();

    CameraParameters(double focal_length,
        const Vector2D & principle_point,
        double baseline)
      : focal_length(focal_length),
      principle_point(principle_point),
      baseline(baseline){}

    Vector2D cam_map (const Vector3D & trans_xyz) const;

    Vector3D stereocam_uvu_map (const Vector3D & trans_xyz) const;

    virtual bool read (std::istream& is){
      is >> focal_length;
      is >> principle_point[0];
      is >> principle_point[1];
      is >> baseline;
      return true;
    }

    virtual bool write (std::ostream& os) const {
      os << focal_length << " ";
      os << principle_point.x() << " ";
      os << principle_point.y() << " ";
      os << baseline << " ";
      return true;
    }

    double focal_length;
    Vector2D principle_point;
    double baseline;
};

/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class  VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSE3Expmap();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  virtual void setToOriginImpl() {
    _estimate = SE3Quat();
  }

  virtual void oplusImpl(const double* update_)  {
    Eigen::Map<const Vector6d> update(update_);
    setEstimate(SE3Quat::exp(update)*estimate());
  }
};

struct Se2 {
  float x, y, theta;
  Se2() {}
  Se2(float _x, float _y ,float _theta) : x(_x), y(_y), theta(_theta) {}
  ~Se2() {}

  Se2 operator -(const Se2& tominus)
  {
  double PI = M_PI;

  float dx = x - tominus.x;
  float dy = y - tominus.y;
  float dtheta = theta - tominus.theta;

  //  Why ???
  dtheta = dtheta - std::floor(dtheta/(2*PI))*2*PI;

  if (dtheta > PI) {
      dtheta -= 2*PI;
  }

  float cost = std::cos(tominus.theta);
  float sint = std::sin(tominus.theta);
  //  Note: dx and dy, which is expressed in world frame,
  //  should be transformed to be expressed in the previous frame
  return Se2(cost*dx+sint*dy, -sint*dx+cost*dy, dtheta);
  }

  Se2 operator +(const Se2& toadd)
  {
  //  Note: dx and dy, which is expressed in the previous,
  //  should be transformed to be expressed in the world frame
  float cost = std::cos(theta);
  float sint = std::sin(theta);
  float _x = x + toadd.x*cost - toadd.y*sint;
  float _y = y + toadd.x*sint + toadd.y*cost;
  float _theta = theta + toadd.theta;
  if(_theta > M_PI) _theta -= 2*M_PI;
  else if(_theta < -M_PI) _theta += 2*M_PI;

  return Se2(_x, _y, _theta);
  }

  cv::Mat toCvSE3()
  {
  float c = std::cos(theta);
  float s = std::sin(theta);

  return (cv::Mat_<float>(4,4) <<
      c,-s, 0, x,
      s, c, 0, y,
      0, 0, 1, 0,
      0, 0, 0, 1);
  }
};

typedef Eigen::Matrix<double,3,1,Eigen::ColMajor> Vector3D;
typedef Eigen::Matrix<double,3,3,Eigen::ColMajor> Matrix3D;
typedef Eigen::Matrix<double,6,1,Eigen::ColMajor> Vector6D;
typedef Eigen::Matrix<double,6,6,Eigen::ColMajor> Matrix6D;

Matrix3D Jl(const Vector3D& v3d);

Matrix3D invJl(const Vector3D& v3d);

Matrix6D AdjTR(const g2o::SE3Quat& pose);
Matrix6D AdjTR(const Eigen::Matrix4d& pose);

//~ ! rho: translation; phi: rotation
//~ ! vector order: [rot, trans]
Matrix6D invJJl(const Vector6D& v6d);

bool verifyInfo(const Matrix6D& info);
bool verifyInfo(const Matrix3D& info);

/**
 * \brief SE(2) Planar Motion Constraints on Camera Pose
 */
 //  _measurement : Tcw (SE3Quat)
 class EdgeSE3ExpmapPrior: public g2o::BaseUnaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap> {
   public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   EdgeSE3ExpmapPrior() : g2o::BaseUnaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap>() {}

   bool read(std::istream& is)
   {  Vector7d meas;
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

   bool write(std::ostream& os) const
   {  SE3Quat cam2world(measurement().inverse());
       for (int i=0; i<7; i++)
         os << cam2world[i] << " ";
       for (int i=0; i<6; i++)
         for (int j=i; j<6; j++){
           os << " " <<  information()(i,j);
         }
       return os.good();
   }

   virtual void computeError();

   virtual void linearizeOplus();
 };
 /// Add SE(2) planeMotion constraint on camera pose Tcw
 EdgeSE3ExpmapPrior* createEdgeSE3ExpmapPrior(const g2o::SE3Quat& Tcv, g2o::VertexSE3Expmap* v_frame);

/**
 * \brief 6D edge between two Vertex6
 */
class  EdgeSE3Expmap : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE3Expmap();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
      const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

      SE3Quat C(_measurement);
      SE3Quat error_= v2->estimate().inverse()*C*v1->estimate();
      _error = error_.log();
    }

    virtual void linearizeOplus();
};

class EdgeProjectXYZ2UV : public  BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ, VertexSE3Expmap>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeProjectXYZ2UV();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
      const CameraParameters * cam
        = static_cast<const CameraParameters *>(parameter(0));
      Vector2D obs(_measurement);
      _error = obs-cam->cam_map(v1->estimate().map(v2->estimate()));
    }

    virtual void linearizeOplus();

    CameraParameters * _cam;
};

//class  EdgeSE2ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE2>
class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(v2->estimate()));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }
    

  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
};



class  EdgeSE3ProjectEcoLine: public  BaseBinaryEdge<1, Vector3d, VertexEcoLinePt2, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectEcoLine();
  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {

      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);//kf

      const VertexEcoLinePt2* v2 = static_cast<const VertexEcoLinePt2*>(_vertices[0]);//line pt2

      const Eigen::Matrix<double,6,1 > &v2Pos = v2->estimate();
      Vector3d pt1, pt2;
      {
          pt1<<v2Pos[0],v2Pos[1],v2Pos[2];
          pt2<<v2Pos[3],v2Pos[4],v2Pos[5];
      }
//      std::cout<<" pt1"<<pt1<<std::endl;
//      std::cout<<" pt2"<<pt2<<std::endl;
//      std::cout<<" v1->estimate()"<<v1->estimate()<<std::endl;

      Vector2d uv1 =cam_project(v1->estimate().map(pt1));
      Vector2d uv2 =cam_project(v1->estimate().map(pt2));
      double a = _measurement[0], b = _measurement[1], c = _measurement[2];

      if(fabs(a*a+ b*b - 1)>0.1 )
      {
          std::cerr<<" abs(a*a+ b*b - 1)>1e-10 "<<a<< " "<<b<<std::endl;
//          exit(-1);
      }

      double dist1 = (a*uv1[0] + b*uv1[1] +c);
      dist1*=dist1;
      double dist2 = (a*uv2[0] + b*uv2[1] +c);
      dist2*=dist2;
      _error << sqrt(dist1+dist2);
//      std::cout<<" "<<sqrt(dist1+dist2)<<std::endl;
  }

//  bool isDepthPositive() {
//    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
//    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
//    return (v1->estimate().map(v2->estimate()))(2)>0.0;
//  }


  virtual void linearizeOplus();

  Vector2d cam_project( const Vector3d & trans_xyz) ;
  double fx, fy, cx, cy;
};



class  EdgeStereoSE3ProjectXYZ: public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(v2->estimate()),bf);
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const;

  double fx, fy, cx, cy, bf;
};

class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(Xw));
    //   _error = obs-cam_project(v1->estimate().toVector());
     //   std::cout<<"_error = "<<_error<<std::endl;
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  Vector3d Xw;
  double fx, fy, cx, cy;
};


class  EdgeStereoSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector3d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector3d cam_project(const Vector3d & trans_xyz) const;

  Vector3d Xw;
  double fx, fy, cx, cy, bf;
};



} // end namespace

#endif
