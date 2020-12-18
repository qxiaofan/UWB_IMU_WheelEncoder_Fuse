#include "../../config.h"
#include "../core/base_edge.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "../types/types_sba.h"
#include "../types/vertex_se2.h"
#include "../types/se2.h"
#include "../types/se3quat.h"

#include <iostream>


class  EdgeSE2ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE2>{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE2ProjectXYZOnlyPose( Eigen::Matrix3d& Rcv1, Eigen::Vector3d& tcv1){
//      Rcv <<  -0.0124,-0.9999,0.0071,
//              0.8870,-0.0143,-0.4615,
//              0.4616,0.0006,0.8872;
//      tcv << 0.0002,-0.0475,-0.0152;
//            Rcv <<   -0.0123882 ,  -0.999917,  0.00713994,
//                     0.887008 , -0.0142511 ,  -0.461507,
//                      0.46156, 0.000594162  ,  0.887158;
//            tcv << 0.000179223,
//                   -0.047535,
//                   -0.0151648;
      Rcv = Rcv1;
      tcv = tcv1;
//      cout << tcv << endl;
//      cout << Rcv << endl;
//      exit(-1);
  }

  bool read(std::istream& is){
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

 bool write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

  void computeError()  {
    const g2o::VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().toVector());
//    _error = obs-cam_project(v1->estimate().map(Xw));
  }

Eigen::Vector2d oneJacobian(const g2o::SE2& _se2,const double delta,const size_t index){
    Eigen::Vector2d err1;
    Eigen::Vector2d err2;
    Eigen::Vector2d pre_z;
    Eigen::Vector3d newPose1 = _se2.toVector();
    Eigen::Vector3d newPose2 = _se2.toVector();
	newPose1(index) -= delta;
    newPose2(index) += delta;
    err1 = _measurement - cam_project(newPose1);
    err2 = _measurement - cam_project(newPose2);
    Eigen::Vector2d ret = (err2 - err1) / (2 * delta);
	return ret;
}

  void linearizeOplus()
  {
/*
      const g2o::VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);
      g2o::SE2 se2 = v1->estimate();
    //_jacobianOplusXi(2,3)
     _jacobianOplusXi.block<2, 1>(0, 0) = oneJacobian(se2,1e-5, 0);
     _jacobianOplusXi.block<2, 1>(0, 1) = oneJacobian(se2, 1e-5, 1);
     _jacobianOplusXi.block<2, 1>(0, 2) = oneJacobian(se2, 1e-5, 2);
*/
           g2o::VertexSE2* v1 = static_cast< g2o::VertexSE2*>(_vertices[0]);

           double x = v1->estimate().translation().x();
           double y = v1->estimate().translation().y();
           double theta = v1->estimate().rotation().angle();

           Eigen::Matrix3d Rz;
           Rz.setIdentity();
           Rz(0,0) = cos(theta);
           Rz(0,1) = -sin(theta);
           Rz(1,0) = sin(theta);
           Rz(1,1) = cos(theta);
           Eigen::Vector3d t;
           t << x,y,0.;
            Eigen::Vector3d VV1 = Rz*Xw+t;
            Eigen::Vector3d VV = Rcv*VV1+tcv;

            Eigen::Matrix<double,2,3> tmp;

            tmp(0,0) = fx;
            tmp(0,1) = 0;
            tmp(0,2) = -VV(0)/VV(2)*fx;

            tmp(1,0) = 0;
            tmp(1,1) = fy;
            tmp(1,2) = -VV(1)/VV(2)*fy;
            tmp = tmp / VV(2);

           Eigen::Matrix<double,2,4> K;
           K.setZero();
           K.block<2,3>(0,0) = tmp;

           Eigen::Matrix<double,2,3> tempVec = tmp * Rcv;

           double cth = cos(theta);
           double sth = sin(theta);

           _jacobianOplusXi(0, 0) = tempVec(0,0);
           _jacobianOplusXi(0, 1) = tempVec(0,1);
           _jacobianOplusXi(0, 2) = (-(sth*Xw(0)+cth*Xw(1)))*tempVec(0,0) + (Xw(0)*cth-Xw(1)*sth)*tempVec(0,1);
           _jacobianOplusXi(1, 0) = tempVec(1,0);
           _jacobianOplusXi(1, 1) = tempVec(1,1);
           _jacobianOplusXi(1, 2) = (-(sth*Xw(0)+cth*Xw(1)))*tempVec(1,0) + (Xw(0)*cth-Xw(1)*sth)*tempVec(1,1);

           _jacobianOplusXi = -_jacobianOplusXi;
   }



  Eigen::Vector2d cam_project(const Eigen::Vector3d & pose2d) //const pose2D= Trw
  {
      float x = pose2d(0);
      float y = pose2d(1);
      float theta= pose2d(2);
      Eigen::Matrix3d Rz;
      Rz.setIdentity();
      Rz(0,0) = cos(theta);
      Rz(0,1) = -sin(theta);
      Rz(1,0) = sin(theta);
      Rz(1,1) = cos(theta);
      Eigen::Vector3d t;
      t << x,y,0.;
       //Eigen::Vector3d VV1 = Rz*Xw+Rz*t;
       Eigen::Vector3d VV1 = Rz*Xw+t;
       Eigen::Vector3d VV = Rcv*VV1+tcv;
       Eigen::Vector3d proj = VV / VV(2);
       Eigen::Vector2d res;
       res[0] = proj[0]*fx + cx;
       res[1] = proj[1]*fy + cy;

       return res;
}

  Eigen::Vector2d project2d(const Eigen::Vector3d& v)
  {
        Eigen::Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
  }

  Eigen::Matrix3d Rcv;
  Eigen::Vector3d tcv;

  Eigen::Vector3d Xw;//WorldPos
  double fx, fy, cx, cy;
};

class  EdgeSE2ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE2>{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE2ProjectXYZ( Eigen::Matrix3d& Rcv1, Eigen::Vector3d& tcv1){
      Rcv = Rcv1;
      tcv = tcv1;
  }

  bool read(std::istream& is){
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

 bool write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


 bool isDepthPositive() {
     const g2o::VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(_vertices[1]);
     g2o::VertexSBAPointXYZ* v0 = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
     Eigen::Vector3d Xw = v0->estimate();
     Eigen::Vector3d  pose2d = v1->estimate().toVector();

     float x = pose2d(0);
     float y = pose2d(1);
     float theta= pose2d(2);
     Eigen::Matrix3d Rz;
     Rz.setIdentity();
     Rz(0,0) = cos(theta);
     Rz(0,1) = -sin(theta);
     Rz(1,0) = sin(theta);
     Rz(1,1) = cos(theta);
     Eigen::Vector3d t;
     t << x,y,0.;
      //Eigen::Vector3d VV1 = Rz*Xw+Rz*t;
      Eigen::Vector3d VV1 = Rz*Xw+t;
      Eigen::Vector3d VV = Rcv*VV1+tcv;
      return VV(2)>0.0;
 }

  void computeError()  {
    const g2o::VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(_vertices[1]);
    g2o::VertexSBAPointXYZ* v0 = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d Xw = v0->estimate();
    Eigen::Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().toVector() , Xw  );
//    _error = obs-cam_project(v1->estimate().map(Xw));
  }

  void linearizeOplus()
  {
       g2o::VertexSE2* v1 = static_cast< g2o::VertexSE2*>(_vertices[1]);
       g2o::VertexSBAPointXYZ* v0 = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
       Eigen::Vector3d Xw = v0->estimate();

       double x = v1->estimate().translation().x();
       double y = v1->estimate().translation().y();
       double theta = v1->estimate().rotation().angle();

       Eigen::Matrix3d Rz;
       Rz.setIdentity();
       Rz(0,0) = cos(theta);
       Rz(0,1) = -sin(theta);
       Rz(1,0) = sin(theta);
       Rz(1,1) = cos(theta);
       Eigen::Vector3d t;
       t << x,y,0.;
        Eigen::Vector3d VV1 = Rz*Xw+t;
        Eigen::Vector3d VV = Rcv*VV1+tcv;

        Eigen::Matrix<double,2,3> tmp;

        tmp(0,0) = fx;
        tmp(0,1) = 0;
        tmp(0,2) = -VV(0)/VV(2)*fx;

        tmp(1,0) = 0;
        tmp(1,1) = fy;
        tmp(1,2) = -VV(1)/VV(2)*fy;
        tmp = tmp / VV(2);

       Eigen::Matrix<double,2,3> tempVec = tmp * Rcv;

       Eigen::Matrix<double,2,3> tmp1;

       tmp1(0,0) = fx;
       tmp1(0,1) = 0;
       tmp1(0,2) = VV(0)/VV(2)*fx;

       tmp1(1,0) = 0;
       tmp1(1,1) = fy;
       tmp1(1,2) = VV(1)/VV(2)*fy;
       tmp1 = tmp1 / VV(2);

      Eigen::Matrix<double,2,3> tempVec1 = tmp1 * Rcv;

       double cth = cos(theta);
       double sth = sin(theta);

//       _jacobianOplusXj(0, 0) = cth * tempVec1(0,0) + sth * tempVec1(0,1);
//       _jacobianOplusXj(0, 1) = -1 * sth * tempVec1(0,0) + cth * tempVec1(0,1);
//       _jacobianOplusXj(0, 2) = tempVec1(0,2);
//       _jacobianOplusXj(1, 0) = cth * tempVec1(1,0) + sth * tempVec1(1,1);
//       _jacobianOplusXj(1, 1) = -1 * sth * tempVec1(1,0) + cth * tempVec1(1,1);
//       _jacobianOplusXj(1, 2) = tempVec1(1,2);
//       _jacobianOplusXj = _jacobianOplusXj;
       _jacobianOplusXi(0, 0) = cth * tempVec1(0,0) + sth * tempVec1(0,1);
       _jacobianOplusXi(0, 1) = -1 * sth * tempVec1(0,0) + cth * tempVec1(0,1);
       _jacobianOplusXi(0, 2) = tempVec1(0,2);
       _jacobianOplusXi(1, 0) = cth * tempVec1(1,0) + sth * tempVec1(1,1);
       _jacobianOplusXi(1, 1) = -1 * sth * tempVec1(1,0) + cth * tempVec1(1,1);
       _jacobianOplusXi(1, 2) = tempVec1(1,2);
       _jacobianOplusXi = _jacobianOplusXi;

       _jacobianOplusXj(0, 0) = tempVec(0,0);
       _jacobianOplusXj(0, 1) = tempVec(0,1);
       _jacobianOplusXj(0, 2) = (-(sth*Xw(0)+cth*Xw(1)))*tempVec(0,0) + (Xw(0)*cth-Xw(1)*sth)*tempVec(0,1);
       _jacobianOplusXj(1, 0) = tempVec(1,0);
       _jacobianOplusXj(1, 1) = tempVec(1,1);
       _jacobianOplusXj(1, 2) = (-(sth*Xw(0)+cth*Xw(1)))*tempVec(1,0) + (Xw(0)*cth-Xw(1)*sth)*tempVec(1,1);

       _jacobianOplusXj = -_jacobianOplusXj;

//       cout << _jacobianOplusXi;
//       cout << _jacobianOplusXj;
   }



  Eigen::Vector2d cam_project(const Eigen::Vector3d & pose2d, Eigen::Vector3d & Xw) //const pose2D= Trw
  {
      float x = pose2d(0);
      float y = pose2d(1);
      float theta= pose2d(2);
      Eigen::Matrix3d Rz;
      Rz.setIdentity();
      Rz(0,0) = cos(theta);
      Rz(0,1) = -sin(theta);
      Rz(1,0) = sin(theta);
      Rz(1,1) = cos(theta);
      Eigen::Vector3d t;
      t << x,y,0.;
       //Eigen::Vector3d VV1 = Rz*Xw+Rz*t;
       Eigen::Vector3d VV1 = Rz*Xw+t;
       Eigen::Vector3d VV = Rcv*VV1+tcv;
       Eigen::Vector3d proj = VV / VV(2);
       Eigen::Vector2d res;
       res[0] = proj[0]*fx + cx;
       res[1] = proj[1]*fy + cy;

       return res;
}

  Eigen::Vector2d project2d(const Eigen::Vector3d& v)
  {
        Eigen::Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
  }

  Eigen::Matrix3d Rcv;
  Eigen::Vector3d tcv;

  double fx, fy, cx, cy;
};

/**
 * \brief 2D edge between two Vertex2
 */
class  EdgeSE2 : public g2o::BaseBinaryEdge<3, g2o::SE2, g2o::VertexSE2, g2o::VertexSE2>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE2()
    {
    }

    void computeError()
    {
      const g2o::VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);
      const g2o::VertexSE2* v2 = static_cast<const g2o::VertexSE2*>(_vertices[1]);
      g2o::SE2 delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
      _error = delta.toVector();
    }
    bool read(std::istream& is)
    {
        Eigen::Vector3d p;
        is >> p[0] >> p[1] >> p[2];
        setMeasurement(g2o::SE2(p));
        _inverseMeasurement = measurement().inverse();
        for (int i = 0; i < 3; ++i)
          for (int j = i; j < 3; ++j) {
            is >> information()(i, j);
            if (i != j)
              information()(j, i) = information()(i, j);
          }
        return true;
    }

    bool write(std::ostream& os) const
    {
        Eigen::Vector3d p = measurement().toVector();
        os << p.x() << " " << p.y() << " " << p.z();
        for (int i = 0; i < 3; ++i)
          for (int j = i; j < 3; ++j)
            os << " " << information()(i, j);
        return os.good();
    }


    virtual void setMeasurement(const g2o::SE2& m){
      _measurement = m;
      _inverseMeasurement = m.inverse();
    }

    virtual bool setMeasurementData(const double* d){
      _measurement=g2o::SE2(d[0], d[1], d[2]);
      _inverseMeasurement = _measurement.inverse();
      return true;
    }

    virtual bool setMeasurementFromState() {
      const g2o::VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);
      const g2o::VertexSE2* v2 = static_cast<const g2o::VertexSE2*>(_vertices[1]);
      _measurement = v1->estimate().inverse()*v2->estimate();
      _inverseMeasurement = _measurement.inverse();
      return true;
    }

    virtual void linearizeOplus()
    {
        const g2o::VertexSE2* vi = static_cast<const g2o::VertexSE2*>(_vertices[0]);
        const g2o::VertexSE2* vj = static_cast<const g2o::VertexSE2*>(_vertices[1]);
        double thetai = vi->estimate().rotation().angle();

        Eigen::Vector2d dt = vj->estimate().translation() - vi->estimate().translation();
        double si=sin(thetai), ci=cos(thetai);

        _jacobianOplusXi(0, 0) = -ci; _jacobianOplusXi(0, 1) = -si; _jacobianOplusXi(0, 2) = -si*dt.x()+ci*dt.y();
        _jacobianOplusXi(1, 0) =  si; _jacobianOplusXi(1, 1) = -ci; _jacobianOplusXi(1, 2) = -ci*dt.x()-si*dt.y();
        _jacobianOplusXi(2, 0) =  0;  _jacobianOplusXi(2, 1) = 0;   _jacobianOplusXi(2, 2) = -1;

        _jacobianOplusXj(0, 0) = ci; _jacobianOplusXj(0, 1)= si; _jacobianOplusXj(0, 2)= 0;
        _jacobianOplusXj(1, 0) =-si; _jacobianOplusXj(1, 1)= ci; _jacobianOplusXj(1, 2)= 0;
        _jacobianOplusXj(2, 0) = 0;  _jacobianOplusXj(2, 1)= 0;  _jacobianOplusXj(2, 2)= 1;

        const g2o::SE2& rmean = _inverseMeasurement;
        Eigen::Matrix3d z = Eigen::Matrix3d::Zero();
        z.block<2, 2>(0, 0) = rmean.rotation().toRotationMatrix();
        z(2, 2) = 1.;
        _jacobianOplusXi = z * _jacobianOplusXi;
        _jacobianOplusXj = z * _jacobianOplusXj;
    }

  protected:
    g2o::SE2 _inverseMeasurement;
};


