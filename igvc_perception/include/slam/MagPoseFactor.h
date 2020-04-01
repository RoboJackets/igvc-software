#ifndef SRC_MAGPOSEFACTOR_H
#define SRC_MAGPOSEFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

class MagPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  const gtsam::Point3 measured_;  ///< The measured magnetometer values
  const gtsam::Point3 nM_;        ///< Local magnetic field (mag output units)
  const gtsam::Point3 bias_;      ///< bias

public:
  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param key of the unknown Pose X in the factor graph
   * @param measured magnetometer reading, a 3-vector
   * @param scale by which a unit vector is scaled to yield a magnetometer reading
   * @param direction of the local magnetic field, see e.g. http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
   * @param bias of the magnetometer, modeled as purely additive (after scaling)
   * @param model of the additive Gaussian noise that is assumed
   */
  MagPoseFactor(gtsam::Key key, const gtsam::Point3& measured, double scale, const gtsam::Unit3& direction,
                const gtsam::Point3& bias, const gtsam::SharedNoiseModel& model)
    : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key)
    ,  //
    measured_(measured)
    , nM_(scale * direction)
    , bias_(bias)
  {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new MagPoseFactor(*this)));
  }

  /**
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& nRb, boost::optional<gtsam::Matrix&> H) const override
  {
    // measured bM = nRbÕ * nM + b
    gtsam::Point3 hx = nRb.rotation().unrotate(nM_, H, boost::none) + bias_;
    // H is 3x3 from above unrotate operation, but needs to be 3x6, ie. set the 3x3 block on the right
    // to all zeros.
    if (H) {
      H->conservativeResize(3, 6);
      H->rightCols(3).setZero();
    }
    const auto error = hx - measured_;
    return error;
//    std::cout << "> [MagPoseFactor] evaluateError\n";
//    std::cout << "         hx: " << hx.transpose() << "\n";
//    std::cout << "  measured_: " << measured_.transpose() << "\n";
//    std::cout << "      error: " << error.transpose() << "\n";
//    const auto whitened = noiseModel_ ? noiseModel_->whiten(error) : error;
//    std::cout << "   whitened: " << whitened.transpose() << std::endl;
//    std::cout << "  tot error: " << whitened.norm() << std::endl;
//    return gtsam::Vector3::Zero();
  }

  void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const override {
      std::cout << s << "MagPoseFactor on " << keyFormatter(key()) << "\n";
      std::cout << s << "  Mag measurement: " << measured_.transpose() << std::endl;
      noiseModel_->print("  noise model: ");
  }
};

#endif  // SRC_MAGPOSEFACTOR_H
