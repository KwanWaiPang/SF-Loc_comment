/**
* This file is modified from DM-VIO, which is written by Vladyslav Usenko for the paper "Direct Visual-Inertial Odometry with Stereo Cameras".
* See https://github.com/lukasvst/dm-vio/blob/master/src/GTSAMIntegration/Marginalization.cpp
*/

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include "CustomMarginalization.h"

gtsam::LinearContainerFactor
gtsam::marginalizeOut(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                      const gtsam::FastVector<gtsam::Key>& keysToMarginalize)
{
    if(keysToMarginalize.empty())
    {
        std::cout << "WARNING: Calling marginalizeOut with empty keysToMarginalize." << std::endl;
        return gtsam::LinearContainerFactor();
    }

    std::shared_ptr<gtsam::NonlinearFactorGraph> newGraph(new gtsam::NonlinearFactorGraph);

    gtsam::NonlinearFactorGraph marginalizedOutGraph;

    gtsam::FastSet<gtsam::Key> setOfKeysToMarginalize(keysToMarginalize);
    gtsam::FastSet<gtsam::Key> connectedKeys;

    extractKeysToMarginalize(graph, *newGraph, marginalizedOutGraph, setOfKeysToMarginalize, connectedKeys);

    gtsam::GaussianFactorGraph::shared_ptr linearizedFactorsToMarginalize = marginalizedOutGraph.linearize(values);
    std::map<gtsam::Key, size_t> keyDimMap = linearizedFactorsToMarginalize->getKeyDimMap();

    int mSize = 0;
    int aSize = 0;

    gtsam::Ordering ordering;

    gtsam::Ordering connectedOrdering;
    gtsam::FastVector<size_t> connectedDims;
    for(const gtsam::Key& k : setOfKeysToMarginalize)
    {
        ordering.push_back(k);
        mSize += keyDimMap[k];
    }
    for(const gtsam::Key& k : connectedKeys)
    {
        ordering.push_back(k);
        connectedOrdering.push_back(k);
        connectedDims.push_back(keyDimMap[k]);
        aSize += keyDimMap[k];
    }

    gtsam::Matrix hessian = linearizedFactorsToMarginalize->augmentedHessian(ordering);

    gtsam::Matrix HAfterSchurComplement = computeSchurComplement(hessian, mSize, aSize);

    gtsam::SymmetricBlockMatrix sm(connectedDims, true);
    sm.setFullMatrix(HAfterSchurComplement);

    gtsam::LinearContainerFactor lcf=gtsam::LinearContainerFactor(
            gtsam::HessianFactor(connectedOrdering, sm), values);

    return lcf;
}

void gtsam::extractKeysToMarginalize(const gtsam::NonlinearFactorGraph& graph, gtsam::NonlinearFactorGraph& newGraph,
                                     gtsam::NonlinearFactorGraph& marginalizedOutGraph,
                                     gtsam::FastSet<gtsam::Key>& setOfKeysToMarginalize,
                                     gtsam::FastSet<gtsam::Key>& connectedKeys)
{
    for(size_t i = 0; i < graph.size(); i++)
    {
        gtsam::NonlinearFactor::shared_ptr factor = graph.at(i);

        gtsam::FastSet<gtsam::Key> set_of_factor_keys(factor->keys());

        gtsam::FastSet<gtsam::Key> intersection;

        std::set_intersection(setOfKeysToMarginalize.begin(), setOfKeysToMarginalize.end(),
                              set_of_factor_keys.begin(), set_of_factor_keys.end(),
                              std::inserter(intersection, intersection.begin()));

        if(!intersection.empty())
        {
            std::set_difference(set_of_factor_keys.begin(), set_of_factor_keys.end(),
                                setOfKeysToMarginalize.begin(), setOfKeysToMarginalize.end(),
                                std::inserter(connectedKeys, connectedKeys.begin()));

            marginalizedOutGraph.add(factor);
        }else
        {
            newGraph.add(factor);
        }
    }
}

gtsam::Matrix gtsam::computeSchurComplement(const gtsam::Matrix& augmentedHessian, int mSize, int aSize)
{
    int n = augmentedHessian.rows() - 1;
    auto pair = std::pair<gtsam::Matrix, gtsam::Vector>(augmentedHessian.block(0, 0, n, n),
                                                        augmentedHessian.block(0, n, n, 1));

    // Preconditioning like in DSO code.
    gtsam::Vector SVec = (pair.first.diagonal().cwiseAbs() +
                          gtsam::Vector::Constant(pair.first.cols(), 10)).cwiseSqrt();
    gtsam::Vector SVecI = SVec.cwiseInverse();

    gtsam::Matrix hessianScaled = SVecI.asDiagonal() * pair.first * SVecI.asDiagonal();
    gtsam::Vector bScaled = SVecI.asDiagonal() * pair.second;

    gtsam::Matrix Hmm = hessianScaled.block(0, 0, mSize, mSize);
    gtsam::Matrix Hma = hessianScaled.block(0, mSize, mSize, aSize);
    gtsam::Matrix Haa = hessianScaled.block(mSize, mSize, aSize, aSize);

    gtsam::Vector bm = bScaled.segment(0, mSize);
    gtsam::Vector ba = bScaled.segment(mSize, aSize);

    // Compute inverse.
    gtsam::Matrix HmmInv = Hmm.completeOrthogonalDecomposition().pseudoInverse();

    gtsam::Matrix HaaNew = Haa - Hma.transpose() * HmmInv * Hma;
    gtsam::Vector baNew = ba - Hma.transpose() * HmmInv * bm;

    // Unscale
    gtsam::Vector SVecUpdated = SVec.segment(mSize, aSize);
    gtsam::Matrix HNewUnscaled = SVecUpdated.asDiagonal() * HaaNew * SVecUpdated.asDiagonal();
    gtsam::Matrix bNewUnscaled = SVecUpdated.asDiagonal() * baNew;

    // Make Hessian symmetric for numeric reasons.
    HNewUnscaled = 0.5 * (HNewUnscaled.transpose() + HNewUnscaled).eval();

    gtsam::Matrix augmentedHRes(aSize + 1, aSize + 1);
    augmentedHRes.setZero();
    augmentedHRes.topLeftCorner(aSize, aSize) = HNewUnscaled;
    augmentedHRes.topRightCorner(aSize, 1) = bNewUnscaled;
    augmentedHRes.bottomLeftCorner(1, aSize) = bNewUnscaled.transpose();
    augmentedHRes(aSize,aSize) = 0.0;

    return augmentedHRes;
}

gtsam::Matrix gtsam::BA2GTSAM(const gtsam::Matrix& H,
                              const gtsam::Vector& v,
                              const gtsam::Pose3& Tbc) {
    gtsam::Matrix A = -Tbc.inverse().AdjointMap();
    gtsam::Matrix Ap = A;
    Ap.block(0,0,3,6) = A.block(3,0,3,6);
    Ap.block(3,0,3,6) = A.block(0,0,3,6);
    
    int ss = H.rows()/6;
    gtsam::Matrix Hnew(ss*6,ss*6+1);
    for(int i = 0; i<ss;i++)
    {
    for(int j = 0; j<ss;j++)
        Hnew.block(i*6,j*6,6,6) = Ap.transpose() * H.block(i*6,j*6,6,6) * Ap;
    Hnew.block(i*6,ss*6,6,1) = Ap.transpose() * v.segment(i*6,6);
    }
    return Hnew;
}

gtsam::Vector gtsam::GTSAM2BA(const gtsam::Vector& x, const gtsam::Pose3& Tbc) {
    gtsam::Matrix A = -Tbc.inverse().AdjointMap();
    gtsam::Matrix Ap = A;
    Ap.block(0,0,3,6) = A.block(3,0,3,6);
    Ap.block(3,0,3,6) = A.block(0,0,3,6);

    int ss = x.rows()/6;
    gtsam::Vector xnew = x;
    for(int i=0;i<ss;i++)
    {
        xnew.segment(i*6,6) = Ap * x.segment(i*6,6);
    }
    return xnew;
}
