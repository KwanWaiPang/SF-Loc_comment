
/**
 * @file InvDepthFactor3.h
 * @brief Inverse Depth Factor based on Civera09tro, Montiel06rss.
 * Landmarks are initialized from the first camera observation with
 * (x,y,z,theta,phi,inv_depth), where x,y,z are the coordinates of
 * the camera. InvDepthCamera provides methods to initialize inverse
 * depth landmarks (backproject), and to convert inverse depth
 * landmarks to cartesian coordinates (Point3) for visualization, etc.
 * The inverse depth parameterization is split into (x,y,z,theta,phi),
 * (inv_depth) to make it easy to add a prior on inverse depth alone
 * @author Chris Beall
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtsam {

/**
 * Ternary factor representing a visual measurement that includes inverse depth
 */
class AnchoredInvDepthFactor: public NoiseModelFactorN<Pose3, Pose3, double> {
protected:

  // Keep a copy of measurement and calibration for I/O
  Point2 measured0_;                ///< 2D measurement
  Point2 measured1_;                ///< 2D measurement
  std::shared_ptr<Cal3_S2> K_;  ///< shared pointer to calibration object

public:

  /// shorthand for base class type
  typedef NoiseModelFactor3<Pose3, Pose3, double> Base;

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;
  
  typedef AnchoredInvDepthFactor This;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor
  AnchoredInvDepthFactor() :
      measured0_(0.0, 0.0),measured1_(0.0,0.0), K_(new Cal3_S2(444, 555, 666, 777, 888)) {
  }

  AnchoredInvDepthFactor(const Point2& measured0, const Point2& measured1, const SharedNoiseModel& model,
      const Key pose0Key, const Key pose1Key, const Key invDepthKey, const Cal3_S2::shared_ptr& K) :
        Base(model, pose0Key, pose1Key, invDepthKey), measured0_(measured0), measured1_(measured1), K_(K) {}

  /** Virtual destructor */
  ~AnchoredInvDepthFactor() override {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "AnchoredInvDepthFactor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured0_, s + ".z");
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) 
    && traits<Point2>::Equals(this->measured0_, e->measured0_, tol) 
    && traits<Point2>::Equals(this->measured1_, e->measured1_, tol) 
    && this->K_->equals(*e->K_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3& pose0, const Pose3& pose1, const double& invDepth,
      OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3) const override {
    try {
      PinholeCamera<Cal3_S2> camera0(pose0, *K_);
      PinholeCamera<Cal3_S2> camera1(pose1, *K_);

      double depth = 1/invDepth;
      double d_depth_invdepth = - 1/ invDepth/invDepth;

      Eigen::Matrix<double,3,6> d_wpf_pose0;
      Eigen::Matrix<double,3,1> d_wpf_depth;
      Eigen::Matrix<double,3,2> Dresult_dp;
      Eigen::Matrix<double,3,5> Dresult_dcal;
      Eigen::Vector3d w_p_f = camera0.backproject(measured0_, depth, d_wpf_pose0, Dresult_dp, d_wpf_depth, Dresult_dcal);

      Eigen::Matrix<double,2,6> d_uv_dpose1;
      Eigen::Matrix<double,2,3> d_uv_wpf;
      Eigen::Matrix<double,2,5> Dcal;
      Eigen::Vector2d uv = camera1.project(w_p_f, d_uv_dpose1, d_uv_wpf, Dcal);

      if(H1) *H1 = d_uv_wpf * d_wpf_pose0;
      if(H2) *H2 = d_uv_dpose1;
      if(H3) *H3 = d_uv_wpf * d_wpf_depth * d_depth_invdepth;

      return uv - measured1_;

    } catch( CheiralityException& e) {
      std::cerr<<"Big problem!!!!"<<std::endl;
      std::cerr<<invDepth<<std::endl;
      std::cerr<<pose0.matrix()<<std::endl;
      std::cerr<<pose1.matrix()<<std::endl;
    }
    if(H1) *H1 = Eigen::Matrix<double,2,6>::Zero();
    if(H2) *H2 = Eigen::Matrix<double,2,6>::Zero();
    if(H3) *H3 = Eigen::Matrix<double,2,1>::Zero();
    return Point2(0,0);
  }

};


class AnchoredFixedInvDepthFactor: public NoiseModelFactorN<Pose3, Pose3> {
protected:

  // Keep a copy of measurement and calibration for I/O
  Point2 measured0_;                ///< 2D measurement
  Point2 measured1_;                ///< 2D measurement
  double idepth_;
  std::shared_ptr<Cal3_S2> K_;  ///< shared pointer to calibration object

public:

  /// shorthand for base class type
  typedef NoiseModelFactor2<Pose3, Pose3> Base;

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;
  
  typedef AnchoredFixedInvDepthFactor This;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor
  AnchoredFixedInvDepthFactor() :
      measured0_(0.0, 0.0),measured1_(0.0,0.0), K_(new Cal3_S2(444, 555, 666, 777, 888)) {
  }

  AnchoredFixedInvDepthFactor(const Point2& measured0, const Point2& measured1, const double&idepth, const SharedNoiseModel& model,
      const Key pose0Key, const Key pose1Key,  const Cal3_S2::shared_ptr& K) :
        Base(model, pose0Key, pose1Key), measured0_(measured0), measured1_(measured1),idepth_(idepth), K_(K) {}

  /** Virtual destructor */
  ~AnchoredFixedInvDepthFactor() override {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "AnchoredFixedInvDepthFactor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured0_, s + ".z");
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) 
    && traits<Point2>::Equals(this->measured0_, e->measured0_, tol) 
    && traits<Point2>::Equals(this->measured1_, e->measured1_, tol) 
    && traits<double>::Equals(this->idepth_, e->idepth_, tol) 
    && this->K_->equals(*e->K_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3& pose0, const Pose3& pose1,
      OptionalMatrixType H1, OptionalMatrixType H2) const override {
    try {
      PinholeCamera<Cal3_S2> camera0(pose0, *K_);
      PinholeCamera<Cal3_S2> camera1(pose1, *K_);
      double invDepth = idepth_;
      double depth = 1/invDepth;
      double d_depth_invdepth = - 1/ invDepth/invDepth;

      Eigen::Matrix<double,3,6> d_wpf_pose0;
      Eigen::Matrix<double,3,1> d_wpf_depth;
      Eigen::Matrix<double,3,2> Dresult_dp;
      Eigen::Matrix<double,3,5> Dresult_dcal;
      Eigen::Vector3d w_p_f = camera0.backproject(measured0_, depth, d_wpf_pose0, Dresult_dp, d_wpf_depth, Dresult_dcal);

      Eigen::Matrix<double,2,6> d_uv_dpose1;
      Eigen::Matrix<double,2,3> d_uv_wpf;
      Eigen::Matrix<double,2,5> Dcal;
      Eigen::Vector2d uv = camera1.project(w_p_f, d_uv_dpose1, d_uv_wpf, Dcal);

      if(H1) *H1 = d_uv_wpf * d_wpf_pose0;
      if(H2) *H2 = d_uv_dpose1;

      return uv - measured1_;

    } catch( CheiralityException& e) {
      std::cerr<<"Big problem!!!!"<<std::endl;
      std::cerr<<idepth_<<std::endl;
      std::cerr<<pose0.matrix()<<std::endl;
      std::cerr<<pose1.matrix()<<std::endl;
    }
    if(H1) *H1 = Eigen::Matrix<double,2,6>::Zero();
    if(H2) *H2 = Eigen::Matrix<double,2,6>::Zero();
    return Point2(0,0);
  }

};
} // \ namespace gtsam
