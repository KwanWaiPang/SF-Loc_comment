/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   VelFactor.h
 *  @author Yuxuan Zhou
 *  @brief  Header file for a Simple Velocity Measurement Factor
 *  @date   November 22, 2023
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

class GTSAM_EXPORT VelFactor: public NoiseModelFactorN<Pose3,Vector3> {

private:

  typedef NoiseModelFactorN<Pose3,Vector3> Base;

  Point3 nT_; ///< Position measurement in cartesian coordinates

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<VelFactor> shared_ptr;

  /// Typedef to this class
  typedef VelFactor This;

  /** default constructor - only use for serialization */
  VelFactor(): nT_(0, 0, 0) {}

  ~VelFactor() override {}

  VelFactor(Key key1, Key key2, const Point3& vb, const SharedNoiseModel& model) :
      Base(model, key1, key2), nT_(vb) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& p, const Vector3&v, OptionalMatrixType H1, OptionalMatrixType H2) const override;

  inline const Point3 & measurementIn() const {
    return nT_;
  }

private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(nT_);
  }
#endif
};

} /// namespace gtsam
