/**
* This file is modified from DM-VIO.
* See https://github.com/lukasvst/dm-vio/blob/master/src/GTSAMIntegration/Marginalization.cpp
*/

#ifndef CUSTOM_MARGINALIZATION_H
#define CUSTOM_MARGINALIZATION_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>

namespace gtsam {

gtsam::LinearContainerFactor
marginalizeOut(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
               const gtsam::FastVector<gtsam::Key>& keysToMarginalize);

// Fills newGraph with factors which are not connected and marginalizedOutGraph with all factors which will be marginalized out,
// Also fills setOfKeysToMarginalize, and connectedKeys.
void extractKeysToMarginalize(const gtsam::NonlinearFactorGraph& graph, gtsam::NonlinearFactorGraph& newGraph,
                              gtsam::NonlinearFactorGraph& marginalizedOutGraph,
                              gtsam::FastSet<gtsam::Key>& setOfKeysToMarginalize,
                              gtsam::FastSet<gtsam::Key>& connectedKeys);

// Compute the Schur complement with the given dimension of marginalized factors and other factors.
gtsam::Matrix computeSchurComplement(const gtsam::Matrix& augmentedHessian, int mSize, int aSize);

gtsam::Matrix BA2GTSAM(const gtsam::Matrix& H,const gtsam::Vector& v, const gtsam::Pose3& Tbc);
gtsam::Vector GTSAM2BA(const gtsam::Vector& x, const gtsam::Pose3& Tbc);
}
#endif //DMVIO_MARGINALIZATION_H
