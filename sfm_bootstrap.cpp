
#include "sfm_bootstrap.h"

#include <stdlib.h>
#include <float.h>

void ConvertFromDistortedPixelCoordsToUndistortedNormalisedPoints(const BNLM::Vector2f* inPts, BNLM::Vector2f* outPoints, int ptCount,
																  const BNLM::Matrix3f intrinsics, const BNLM::Vector2f distK) {
	
	BNLM::Matrix3f invIntrinsics = BNLM::Matrix3f::Identity();
	invIntrinsics(0, 0) = 1.0f / intrinsics(0, 0);
	invIntrinsics(1, 1) = 1.0f / intrinsics(1, 1);
	invIntrinsics(0, 2) = intrinsics(0, 2) * -invIntrinsics(0, 0);
	invIntrinsics(1, 2) = intrinsics(1, 2) * -invIntrinsics(1, 1);

	BNS_FOR_I(ptCount) {
		BNLM::Vector2f normalisedDistorted = (invIntrinsics * inPts[i].homo()).subvec<2>(0);

		float r2 = normalisedDistorted.SquareMag();
		// TODO: tangential distotion (P1 + P2) lol
		// Undistorted -> distorted conversion here
		BNLM::Vector2f normalisedUndistorted;
		normalisedUndistorted.x() = normalisedDistorted.x() / (1.0f + distK(0) * r2 + distK(1) * BNS_SQR(r2));
		normalisedUndistorted.y() = normalisedDistorted.y() / (1.0f + distK(0) * r2 + distK(1) * BNS_SQR(r2));

		outPoints[i] = normalisedUndistorted;
	}
}

// TODO: Pull out inot utilities file?
void ShuffleInitialSubsetsInSameWay(Vector<BNLM::Vector2f>* pts1, Vector<BNLM::Vector2f>* pts2, int N) {
	ASSERT(pts1->count == pts2->count);
	ASSERT(pts1->count >= N);

	BNS_FOR_I(N) {
		// MARK: rng
		int randomIndexPastI = i + (rand() % (pts1->count - i));
		ASSERT(randomIndexPastI >= i);
		ASSERT(randomIndexPastI < pts1->count);

		// Maintain the pair-wise associations between the two lists
		BNS_SWAP_VAL(pts1->data[i], pts1->data[randomIndexPastI]);
		BNS_SWAP_VAL(pts2->data[i], pts2->data[randomIndexPastI]);
	}
}

void GenerateFundamentalHypothesisFrom8Points(const BNLM::Vector2f* pts0, const BNLM::Vector2f* pts1, BNLM::Matrix3f* outFundamental) {
	BNLM::Matrix<float, 8, 9> Fsystem;
	Fsystem.ZeroOut();

	BNS_FOR_I(8) {
		float x0 = pts0[i].x();
		float x1 = pts1[i].x();
		float y0 = pts0[i].y();
		float y1 = pts1[i].y();

		Fsystem(i, 0) = x0 * x1;
		Fsystem(i, 1) = x0 * y1;
		Fsystem(i, 2) = x0;
		Fsystem(i, 3) = y0 * x1;
		Fsystem(i, 4) = y0 * y1;
		Fsystem(i, 5) = y0;
		Fsystem(i, 6) = x1;
		Fsystem(i, 7) = y1;
		Fsystem(i, 8) = 1.0f;
	}

	BNLM::Matrix<float, 8, 8> matU; // TODO: Unused, skip computation
	BNLM::Matrix<float, 9, 9> matV;
	BNLM::Vector<float, 9> singularVals;
	BNLM::SingularValueDecomposition(Fsystem, &matU, &singularVals, &matV);

	BNLM::Matrix3f F;
	BNS_FOR_I(3) {
		BNS_FOR_J(3) {
			F(j, i) = matV(j * 3 + i, 8);
		}
	}

	// TODO: Further refinement by ensuring rank 2

	*outFundamental = F;
}

// TODO: Compute inliers/error for fundamental hypothesis
float CalculateErrorForFundamentalMatrix(BNLM::Vector2f* pts0, BNLM::Vector2f* pts1, int pointCount, BNLM::Matrix3f F) {
	// Uhhhh...
	const float maxError = 0.5f;

	float totalErr = 0.0f;
	BNS_FOR_I(pointCount) {
		BNLM::Vector3f pt0 = pts0[i].homo();
		BNLM::Vector3f pt1 = pts1[i].homo();

		BNLM::Vector3f pt1Epi = F * pt1;

		float error = BNLM::DotProduct(pt0, pt1Epi);
		error = BNS_ABS(error);
		error = BNS_MIN(error, maxError);
		totalErr += error;
	}

	return totalErr;
}


// NOTE: Deliberately copy these, so we can modify them in place easily
int CalculateFundamentalMatrixUsing8PointAlgorithm(Vector<BNLM::Vector2f> pointsSrc,
												   Vector<BNLM::Vector2f> pointsDst,
												   BNLM::Matrix3f* outFundamental) {
	//

	ASSERT(pointsSrc.count == pointsDst.count);
	ASSERT(pointsSrc.count >= 8);

	const int ransacIterationCount = 2500;

	BNLM::Matrix3f bestFundamental;
	float bestError = FLT_MAX;

	// TODO: Compute inliers?

	BNS_FOR_I(ransacIterationCount) {
		ShuffleInitialSubsetsInSameWay(&pointsSrc, &pointsDst, 8);
		BNLM::Matrix3f FHypothesis;
		GenerateFundamentalHypothesisFrom8Points(pointsSrc.data, pointsDst.data, &FHypothesis);

		float thisErr = CalculateErrorForFundamentalMatrix(pointsSrc.data, pointsDst.data, pointsSrc.count, FHypothesis);
		if (thisErr < bestError) {
			bestFundamental = FHypothesis;
			bestError = thisErr;
		}
	}

	CalculateErrorForFundamentalMatrix(pointsSrc.data, pointsDst.data, pointsSrc.count, bestFundamental);

	*outFundamental = bestFundamental;

	return 0;
}

// translation and scale are for converting back form normalised points to standard pixel coordinates
// i.e. norm_pt * scale + translation = pixel coords
void NormalisePointsAroundOrigin(Vector<BNLM::Vector2f>* points, BNLM::Vector2f* outTranslation, float* outScale) {
	BNLM::Vector2f centroid = BNLM::Vector2f::Zero();
	BNS_VEC_FOREACH(*points) {
		
		centroid += *ptr;
	}

	centroid /= points->count;

	float avgDistFromCentre = 0.0f;
	BNS_VEC_FOREACH(*points) {
		*ptr -= centroid;
		avgDistFromCentre += ptr->Mag();
	}

	avgDistFromCentre /= points->count;

	// We want an average of sqrt(2) distance around the centre for numerical stability
	float invDistFromCentre = sqrtf(2.0f) / avgDistFromCentre;
	BNS_VEC_FOREACH(*points) {
		*ptr *= invDistFromCentre;
	}

	*outTranslation = centroid;
	*outScale = avgDistFromCentre;
}

int CalculateEssentialMatrixUsing8PointAlgorithm(const Vector<BNFastKeyPoint>& pointsSrc,
												 const Vector<BNFastKeyPoint>& pointsDst,
												 const Vector<BNFeatureMatch>& matches,
												 const BNLM::Matrix3f intrinsics,
												 const BNLM::Vector2f distK,
												 BNLM::Matrix3f* outFundamental) {
	//
	ASSERT(matches.count >= 8);

	Vector<BNLM::Vector2f> points2DSrc;
	Vector<BNLM::Vector2f> points2DDst;

	points2DSrc.EnsureCapacity(pointsSrc.count);
	points2DDst.EnsureCapacity(pointsDst.count);

	BNS_VEC_FOREACH(matches) {
		points2DSrc.PushBack(pointsSrc.data[ptr->srcIdx].imagePoint);
		points2DDst.PushBack(pointsDst.data[ptr->targetIdx].imagePoint);
	}

	// Multiply out intrinsics (and remove distortion while we're at it), so we can directly calculate the essential matrix
	ConvertFromDistortedPixelCoordsToUndistortedNormalisedPoints(points2DSrc.data, points2DSrc.data, points2DSrc.count, intrinsics, distK);
	ConvertFromDistortedPixelCoordsToUndistortedNormalisedPoints(points2DDst.data, points2DDst.data, points2DDst.count, intrinsics, distK);

	// Normalise points so that they're centred at (0,0) and have an average distance of sqrt(2)
	// We do this in the name of numerical stability
	BNLM::Vector2f transSrc, transDst;
	float scaleSrc, scaleDst;
	NormalisePointsAroundOrigin(&points2DSrc, &transSrc, &scaleSrc);
	NormalisePointsAroundOrigin(&points2DDst, &transDst, &scaleDst);

	// Lol, it says fundamental but really it's the essential (cause we multiplied out the intrinsics)
	// TODO: Fix this mess at some point when I'm not tired
	BNLM::Matrix3f normalisedFundamental;
	int ret = CalculateFundamentalMatrixUsing8PointAlgorithm(points2DSrc, points2DDst, &normalisedFundamental);

	// TODO: Rename from pixel, since it's not pixel anymore... :p
	BNLM::Matrix3f dstPixelToNormCoords = BNLM::Matrix3f::Identity();
	BNLM::Matrix3f srcPixelToNormCoords = BNLM::Matrix3f::Identity();

	dstPixelToNormCoords(0, 0) = 1.0f / scaleDst;
	dstPixelToNormCoords(1, 1) = 1.0f / scaleDst;
	dstPixelToNormCoords.block<2, 1>(0, 2) = transDst / -scaleDst;

	srcPixelToNormCoords(0, 0) = 1.0f / scaleSrc;
	srcPixelToNormCoords(1, 1) = 1.0f / scaleSrc;
	srcPixelToNormCoords.block<2, 1>(0, 2) = transSrc / -scaleSrc;

	// TODO: Rename...sdkdgjbnaejksgbbjkg
	*outFundamental = srcPixelToNormCoords.transpose() * normalisedFundamental * dstPixelToNormCoords;//  * normalisedFundamental * srcNormToPixelCoords;

	return ret;
}

int TriangulateNormalisedImagePoints(const BNLM::Vector2f* points1, const BNLM::Vector2f* points2, int ptCount,
									 const BNLM::Matrix4f world2Camera1, const BNLM::Matrix4f world2Camera2,
									 const float fx, // Used for error scaling
									 bool* outWasTriangulated, BNLM::Vector3f* outPoints) {
	// SDGJSG

	int successCount = 0;

	const float reprojectionErrorPixels = 10.0f;
	float reprojectionErrorNormalised = reprojectionErrorPixels / fx;
	float reprojectionSqErrNormalised = BNS_SQR(reprojectionErrorNormalised);

	BNS_FOR_I(ptCount) {
		BNLM::Matrix4f A;
		A.block<1, 4>(0, 0) = world2Camera1.row(2) * points1[i].x() - world2Camera1.row(0);
		A.block<1, 4>(1, 0) = world2Camera1.row(2) * points1[i].y() - world2Camera1.row(1);
		A.block<1, 4>(2, 0) = world2Camera2.row(2) * points2[i].x() - world2Camera2.row(0);
		A.block<1, 4>(3, 0) = world2Camera2.row(2) * points2[i].y() - world2Camera2.row(1);

		BNLM::Matrix4f u, v;
		BNLM::Vector<float, 4> vals;
		BNLM::SingularValueDecomposition(A, &u, &vals, &v);

		// TODO: Potential div by 0?
		BNLM::Vector3f worldPt = v.column(3).hnorm();

		outWasTriangulated[i] = false;

		BNLM::Vector3f camSpace1 = (world2Camera1 * worldPt.homo()).hnorm();
		if (camSpace1.z() <= 0.0f) { continue; }
		BNLM::Vector3f camSpace2 = (world2Camera2 * worldPt.homo()).hnorm();
		if (camSpace2.z() <= 0.0f) { continue; }

		BNLM::Vector2f screenSpace1 = camSpace1.hnorm();
		BNLM::Vector2f screenSpace2 = camSpace2.hnorm();

		float sqrReproErr1 = (points1[i] - screenSpace1).SquareMag();
		float sqrReproErr2 = (points2[i] - screenSpace2).SquareMag();
	
		if (sqrReproErr1 > reprojectionSqErrNormalised) { continue; }
		if (sqrReproErr2 > reprojectionSqErrNormalised) { continue; }

		outWasTriangulated[i] = true;
		successCount++;

		outPoints[i] = worldPt;
	}

	return successCount;
}

void DecomposeEssentialMatrixInto4MotionHypotheses(const BNLM::Matrix3f essential, BNLM::Matrix3f* outRotations, BNLM::Vector3f* outTranslations) {

	BNLM::Matrix3f U, V;
	BNLM::Vector3f sigma;

	BNLM::SingularValueDecomposition(essential, &U, &sigma, &V);

	// TODO: Force sigma to be (s,s,0)?

	// TODO: Whyyyy
	//BNLM::Vector3f t = U.block<3, 1>(0, 2);
	auto blk = U.block<3, 1>(0, 2);
	BNLM::Vector3f t = BNLM::Vector3f(blk(0,0), blk(1, 0), blk(2, 0));
	t.Normalize();

	BNLM::Matrix3f W = BNLM::Matrix3f::Zero();
	W(0, 1) = -1;
	W(1, 0) = 1;
	W(2, 2) = 1;

	BNLM::Matrix3f R1 = U * W * V.transpose();
	BNLM::Matrix3f R2 = U * W.transpose() * V.transpose();

	// TODO: Check determinant and flip stuff if needed...I think?

	if (R1.determinant() < 0) {
		R1 = R1 * -1.0f;
	}

	if (R2.determinant() < 0) {
		R2 = R2 * -1.0f;
	}

	// Four possible combinations of (R1|R2) and (t|-t)
	BNS_FOR_I(4) {
		outRotations[i] = (i < 2) ? R1 : R2;
		outTranslations[i] = (i & 1) ? t : (t * -1.0f);
	}
}



