#ifndef SFM_BOOTSTRAP_H_
#define SFM_BOOTSTRAP_H_

#pragma once

#include "external/BNLM/CppUtils/vector.h"
#include "external/BNLM/core.h"
#include "features.h"
#include "feature_matching.h"

int CalculateFundamentalMatrixUsing8PointAlgorithm(const Vector<BNFastKeyPoint>& pointsSrc,
												   const Vector<BNFastKeyPoint>& pointsDst,
												   const Vector<BNFeatureMatch>& matches,
												   BNLM::Matrix3f* outFundamental);


// TODO: Fundamental matrix -> essential matrix

// TODO: Decompose essential matrix into 4 motion hypotheses

// TODO(other file): Triangulate 2D correspondences from camera poses + intrinsics

// TODO(other file): Linear least-squares batch optimisation lol

#endif
