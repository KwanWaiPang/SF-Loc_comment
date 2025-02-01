#include "VelFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void VelFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "VelFactor on " << keyFormatter(key<1>())<<" "<< keyFormatter(key<2>())
       << "\n";
  cout << "  VelFactor measurement: " << nT_ << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool VelFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(nT_, e->nT_, tol);
}

//***************************************************************************
Vector VelFactor::evaluateError(const Pose3& p, const Vector3& v,
    OptionalMatrixType H1, OptionalMatrixType H2) const {

    Matrix36 tH1;
    Matrix33 tH2;
    Matrix33 tH3;
    
    Rot3 Rwb = p.rotation(tH1);
    Rot3 Rbw = Rwb.inverse(tH2);
    Vector3 vb = Rbw.rotate(v,tH3);
    if(H1) (*H1) = tH3 * tH2 * tH1;
    if(H2) (*H2) = Rbw.matrix();
  return vb -nT_;
}


}/// namespace gtsam
