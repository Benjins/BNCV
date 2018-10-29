#ifndef SFM_BOOTSTRAP_H_
#define SFM_BOOTSTRAP_H_

#pragma once

#include "external/BNLM/CppUtils/vector.h"
#include "external/BNLM/core.h"
#include "features.h"
#include "feature_matching.h"

// TODO: 5-point algo for directly getting essential matrix?

// TODO: Use intrinsics + distortion for...anything?

// TODO: Move this to camera functions file
void ConvertFromDistortedPixelCoordsToUndistortedNormalisedPoints(const BNLM::Vector2f* inPts, BNLM::Vector2f* outPoints, int ptCount,
																  const BNLM::Matrix3f intrinsics, const BNLM::Vector2f distK);

int CalculateEssentialMatrixUsing8PointAlgorithm(const Vector<BNFastKeyPoint>& pointsSrc,
												 const Vector<BNFastKeyPoint>& pointsDst,
												 const Vector<BNFeatureMatch>& matches,
												 const BNLM::Matrix3f intrinsics,
												 const BNLM::Vector2f distK,
												 BNLM::Matrix3f* outFundamental);

// Translations will be normalised I swear
void DecomposeEssentialMatrixInto4MotionHypotheses(const BNLM::Matrix3f essential, BNLM::Matrix3f* outRotations, BNLM::Vector3f* outTranslations);

// TODO: Fundamental matrix -> essential matrix

// TODO: Decompose essential matrix into 4 motion hypotheses

// TODO(other file): Triangulate 2D correspondences from camera poses + intrinsics

int TriangulateNormalisedImagePoints(const BNLM::Vector2f* points1, const BNLM::Vector2f* points2, int ptCount,
									 const BNLM::Matrix4f world2Camera1, const BNLM::Matrix4f world2Camera2,
									 const float fx, // Used for error scaling
									 bool* outWasTriangulated, BNLM::Vector3f* outPoints);


// TODO(other file): Linear least-squares batch optimisation lol

#endif
