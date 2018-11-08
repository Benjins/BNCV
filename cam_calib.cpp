#include "external/BNLM/core.h"

#include "imgproc.h"

struct CameraCalibrationSolverImage {
	// world -> camera
	BNLM::Vector3f rotation;
	// world -> camera
	BNLM::Vector3f translation;

	Vector<CheckerboardCorner> checkerboardPoints;
};

struct CameraCalibrationSolverSystem {
	Vector<CameraCalibrationSolverImage> solverImages;

	float fx, fy;
	float cx, cy;

	BNLM::Vector2f distK;
	BNLM::Vector2f distP;
};

void FindInitialCheckerboardCorners(const Vector<HoughLocalMaximum>& verticalLines,
	const Vector<HoughLocalMaximum>& horizontalLines,
	Vector<CheckerboardCorner>* outCorners) {
	outCorners->Clear();
	outCorners->EnsureCapacity(verticalLines.count * horizontalLines.count);

	// Okay, new plan: since previously we were implictly using point-slope formula,
	// and since the slope is well...infinite...
	// we are now using 2-point x 2-point intersection (formula from wikipedia)
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
	// First we get two points on each line

	BNS_VEC_FOR_I(horizontalLines) {

		HoughLocalMaximum horizontalLine = horizontalLines.data[i];
		float thetaDegrees1 = horizontalLine.thetaDegrees;
		float rho1 = horizontalLine.rho;
		float theta1 = thetaDegrees1 * BNS_DEG2RAD;

		float a1 = cosf(theta1), b1 = sinf(theta1);
		float x1 = a1 * rho1, y1 = b1 * rho1;
		float x2 = x1 - 10 * b1;
		float y2 = y1 + 10 * a1;

		BNS_VEC_FOR_J(verticalLines) {
			HoughLocalMaximum verticalLine = verticalLines.data[j];
			int thetaDegrees2 = verticalLine.thetaDegrees;
			float rho2 = verticalLine.rho;
			float theta2 = thetaDegrees2 * BNS_DEG2RAD;

			float a2 = cosf(theta2), b2 = sinf(theta2);
			float x3 = a2 * rho2, y3 = b2 * rho2;
			float x4 = x3 - 10 * b2;
			float y4 = y3 + 10 * a2;

			float denom = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

			float imageX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;

			float imageY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

			// TODO: Bounds check
			CheckerboardCorner pt;
			pt.imagePt = BNLM::Vector2f(imageX, imageY);
			pt.planePt = BNLM::Vector2f(i, j);
			outCorners->PushBack(pt);
		}
	}
}

inline float LerpFloat(float a, float b, float t) {
	return a * (1.0f - t) + b * t;
}

inline float BilinearSampleOnImage(BNImage<unsigned char> img, BNLM::Vector2f pt) {
	int x0 = (int)pt.x();
	int y0 = (int)pt.y();

	float fracX = pt.x() - x0;
	float fracY = pt.y() - y0;

	float val00 = *img.GetPixelPtr(x0, y0);
	float val01 = *img.GetPixelPtr(x0, y0 + 1);
	float val10 = *img.GetPixelPtr(x0 + 1, y0);
	float val11 = *img.GetPixelPtr(x0 + 1, y0 + 1);

	float val0 = LerpFloat(val00, val01, fracY);
	float val1 = LerpFloat(val10, val11, fracY);
	return LerpFloat(val0, val1, fracX) / 255.0f;
}

void RefineCheckerboardCornerPositionsInImage(BNImage<unsigned char> img, const int searchSize, Vector<CheckerboardCorner>* corners) {

	const int searchSquareSize = 2 * searchSize + 1;

	BNImage<float> gradScratch(searchSquareSize, searchSquareSize);
	BNImage<float> angleScratch(searchSquareSize, searchSquareSize);

	// TODO: Some code dup and some over-use of dynamic buffers... :p
	BNS_VEC_FOREACH(*corners) {
		int xStart = BNS_CLAMP(BNS_ROUND(ptr->imagePt.x() - searchSize), 0, img.width - 1);
		int xEnd = BNS_CLAMP(BNS_ROUND(ptr->imagePt.x() + searchSize), 0, img.width - 1);
		int yStart = BNS_CLAMP(BNS_ROUND(ptr->imagePt.y() - searchSize), 0, img.height - 1);
		int yEnd = BNS_CLAMP(BNS_ROUND(ptr->imagePt.y() + searchSize), 0, img.height - 1);

		auto subImg = img.GetSubImage(xStart, yStart, xEnd - xStart + 1, yEnd - yStart + 1);

		SobelResponseOnImage(subImg, &gradScratch, &angleScratch);
		SobelResponseNonMaxFilter(gradScratch, angleScratch);

		const int thetaResolutionPerDegree = 10;
		BNImage<short> sobelHoughVoting;
		HoughTransformAfterSobel(gradScratch, angleScratch, thetaResolutionPerDegree, &sobelHoughVoting);

		Vector<HoughLocalMaximum> houghLocalMaxima;
		FindLocalMaximaInHoughTransform(sobelHoughVoting, thetaResolutionPerDegree, &houghLocalMaxima, 2, 5);

		Vector<HoughLocalMaximum> verticalLines, horizontalLines;
		FilterAndSortVerticalAndHorizontalHoughBuckets(houghLocalMaxima, &verticalLines, &horizontalLines);

		BNS_VEC_DUMB_SORT(verticalLines, l.score > r.score);
		BNS_VEC_DUMB_SORT(horizontalLines, l.score > r.score);

		if (verticalLines.count > 0 && horizontalLines.count > 0) {
			verticalLines.Resize(1);
			horizontalLines.Resize(1);

			Vector<CheckerboardCorner> checkerboardCorners;
			FindInitialCheckerboardCorners(verticalLines, horizontalLines, &checkerboardCorners);

			ASSERT(checkerboardCorners.count == 1);

			ptr->imagePt = checkerboardCorners.data[0].imagePt + BNLM::Vector2f(xStart, yStart);
		}
	}
}

// We give an initial estimate for the checkerboard corner, and now look for a better one
// in a local radius (since the initial guess didn't take distortion into account)
// TODO: Like...any bounds checking?
void RefineCheckerboardCornerPositionsInImageSubpixel(BNImage<unsigned char> img, const int searchSize, Vector<CheckerboardCorner>* corners) {

	const int searchSquareSize = searchSize * 2 + 1;

	BNImage<float> gradMask(searchSquareSize, searchSquareSize);

	BNS_FOR_J(searchSquareSize) {
		float yOff = j - searchSize;
		float yMask = exp(-BNS_SQR(yOff));
		BNS_FOR_I(searchSquareSize) {
			float xOff = i - searchSize;
			float xMask = exp(-BNS_SQR(xOff));
			*gradMask.GetPixelPtr(i, j) = xMask * yMask;
		}
	}

	BNImage<float> scratchVals(searchSquareSize + 2, searchSquareSize + 2);

	BNS_VEC_FOREACH(*corners) {
		// TODO:

		BNLM::Vector2f currentGuess = ptr->imagePt;

		int iter = 0;
		const int maxIters = 20;

		while (iter < maxIters) {

			// Set up scratchVals w/ subpixel values
			// Because we're doing bilinear filtering, we need an extra pixel border
			// around the whole chunk
			BNS_FOR_J(searchSquareSize + 2) {
				float yOff = j - searchSize - 1;
				BNS_FOR_I(searchSquareSize + 2) {
					float xOff = i - searchSize - 1;
					BNLM::Vector2f samplePt = currentGuess + BNLM::Vector2f(xOff, yOff);
					*scratchVals.GetPixelPtr(i, j) = BilinearSampleOnImage(img, samplePt);
				}
			}

			// | a b |
			// | b c |  is the gradient...jacobian?
			double a = 0, b = 0, c = 0, bb1 = 0, bb2 = 0;

			BNImage<float> scratchValsSubImg = scratchVals.GetSubImage(1, 1, scratchVals.width - 2, scratchVals.height - 2);

			BNS_FOR_J(searchSquareSize) {
				float py = j - searchSize;
				BNS_FOR_I(searchSquareSize) {
					float px = i - searchSize;

					//
					double m = *gradMask.GetPixelPtr(i, j);

					double tgx = *scratchValsSubImg.GetPixelPtrNoABC(i + 1, j) - *scratchValsSubImg.GetPixelPtrNoABC(i - 1, j);
					double tgy = *scratchValsSubImg.GetPixelPtrNoABC(i, j + 1) - *scratchValsSubImg.GetPixelPtrNoABC(i, j - 1);

					double gxx = tgx * tgx * m;
					double gxy = tgx * tgy * m;
					double gyy = tgy * tgy * m;

					a += gxx;
					b += gxy;
					c += gyy;

					bb1 += gxx * px + gxy * py;
					bb2 += gxy * px + gyy * py;
				}
			}

			double determinant = a * c - b * b;

			if (BNS_ABS(determinant) <= 0.000001f) {
				break;
			}

			double scale = 1.0 / determinant;

			// TODO: Uhhh..doubles and floats??
			//BNLM::Vector2f guessMovement(c * bb1 - b * bb2, -b * bb1 + a * bb2);

			float newX = (float)(0.0f + c * scale*bb1 - b * scale*bb2);
			float newY = (float)(0.0f - b * scale*bb1 + a * scale*bb2);

			BNLM::Vector2f guessMovement(newX, newY);
			float err = guessMovement.SquareMag();

			if (err <= 0.0000001f) {
				break;
			}

			currentGuess = currentGuess + guessMovement;

			iter++;
		}

		float totalChangeSqr = (ptr->imagePt - currentGuess).SquareMag();

		// TODO: Parameterise?
		if (totalChangeSqr < 16.0f) {
			ptr->imagePt = currentGuess;
		}
	}
}

BNLM::Matrix3f ComputeHomographyFromPointMatches(const BNLM::Vector2f* pts1, const BNLM::Vector2f* pts2, const int ptCount) {

	int rowCount = 2 * ptCount;

	BNLM::MatrixXf solverSystem(rowCount, 9);
	solverSystem.ZeroOut();

	BNS_FOR_I(ptCount) {

		float u1 = pts1[i].x(), v1 = pts1[i].y();
		float u2 = pts2[i].x(), v2 = pts2[i].y();

		int r1 = i * 2, r2 = i * 2 + 1;

		solverSystem(r1, 0) = u2;
		solverSystem(r1, 1) = v2;
		solverSystem(r1, 2) = 1;

		solverSystem(r2, 3) = u2;
		solverSystem(r2, 4) = v2;
		solverSystem(r2, 5) = 1;

		solverSystem(r1, 6) = -u1 * u2;
		solverSystem(r1, 7) = -u1 * v2;
		solverSystem(r1, 8) = -u1;

		solverSystem(r2, 6) = -u2 * v1;
		solverSystem(r2, 7) = -v1 * v2;
		solverSystem(r2, 8) = -v1;

	}

	BNLM::MatrixXf U, V;
	BNLM::VectorXf sigma;
	BNLM::SingularValueDecomposition(solverSystem, &U, &sigma, &V);

	ASSERT(V.cols == 9);
	ASSERT(V.rows == 9);

	BNLM::Matrix3f H;

	BNS_FOR_J(3) {
		BNS_FOR_I(3) {
			H(j, i) = V(j * 3 + i, 8);
		}
	}

	return H;
}

BNLM::Matrix3f ComputeHomographyFromCheckerboardCorners(const Vector<CheckerboardCorner>& corners, const float scaleFactor) {
	Vector<BNLM::Vector2f> pts1;
	Vector<BNLM::Vector2f> pts2;

	pts1.EnsureCapacity(corners.count);
	pts2.EnsureCapacity(corners.count);

	BNS_VEC_FOR_I(corners) {
		float u1 = corners.data[i].imagePt.x() / scaleFactor, v1 = corners.data[i].imagePt.y() / scaleFactor;
		float u2 = corners.data[i].planePt.x(), v2 = corners.data[i].planePt.y();

		pts1.PushBack(BNLM::Vector2f(u1, v1));
		pts2.PushBack(BNLM::Vector2f(u2, v2));
	}

	ASSERT(pts1.count == pts2.count);

	return ComputeHomographyFromPointMatches(pts1.data, pts2.data, pts1.count);
}

BNLM::Matrix3f ComputeInitialIntrinsicsMatrixFromHomographies(const Vector<BNLM::Matrix3f>& homographies, const float scaleFactor) {

	BNLM::MatrixXf BSystem(2 * homographies.count, 6);
	BSystem.ZeroOut();

	BNS_VEC_FOR_I(homographies) {
		int row1 = 2 * i;
		int row2 = 2 * i + 1;

		auto H = homographies.data[i];

		// Setup the constraints for each row:
		// We have two constraints per homography (unit-length constraint and orthoganl constraint)
		#define SETUP_VEC(vec, i, j) do {                   \
			vec(0) = H(0, i) * H(0, j);                     \
			vec(1) = H(0, i) * H(1, j) + H(1, i) * H(0, j); \
			vec(2) = H(1, i) * H(1, j);                     \
			vec(3) = H(2, i) * H(0, j) + H(0, i) * H(2, j); \
			vec(4) = H(2, i) * H(1, j) + H(1, i) * H(2, j); \
			vec(5) = H(2, i) * H(2, j);                     \
		} while(0)

		{
			BNLM::Vector<float, 6> scratchVec1;
			SETUP_VEC(scratchVec1, 0, 1);
			BNS_FOR_I(6) { BSystem(row1, i) = scratchVec1(i); }
		}

		{
			BNLM::Vector<float, 6> scratchVec1, scratchVec2;
			SETUP_VEC(scratchVec1, 0, 0);
			SETUP_VEC(scratchVec2, 1, 1);
			BNS_FOR_I(6) { BSystem(row2, i) = scratchVec1(i) - scratchVec2(i); }
		}
	}

	BNLM::MatrixXf U, V;
	BNLM::VectorXf sigma;
	BNLM::SingularValueDecomposition(BSystem, &U, &sigma, &V);

	ASSERT(V.rows == 6);
	ASSERT(V.cols == 6);

	BNLM::Vector<float, 6> BVec;
	BNS_FOR_I(6) {
		BVec(i) = V(i, 5);
	}

	BNLM::Matrix3f B;
	B(0, 0) = BVec(0);
	B(0, 1) = BVec(1);
	B(1, 0) = BVec(1);
	B(1, 1) = BVec(2);
	B(0, 2) = BVec(3);
	B(2, 0) = BVec(3);
	B(1, 2) = BVec(4);
	B(2, 1) = BVec(4);
	B(2, 2) = BVec(5);

	printf("Initial estimates of intrinsics:\n");

	float cy = (-B(0, 0) * B(1, 2)) / (B(0, 0) * B(1, 1));
	printf("cy: %f\n", cy * scaleFactor);

	float lambda = B(2, 2) - (BNS_SQR(B(0, 2)) + cy * (B(0, 1) * B(0, 2) - B(0, 0) * B(1, 2))) / B(0, 0);
	float fx = sqrt(lambda / B(0, 0));
	printf("fx: %f\n", fx * scaleFactor);

	float fy = sqrt(lambda * B(0, 0) / (B(0, 0) * B(1, 1) - BNS_SQR(B(0, 1))));
	printf("fy: %f\n", fy * scaleFactor);

	float cx = -B(0, 2) * fx*fx / lambda;
	printf("cx: %f\n", cx * scaleFactor);

	BNLM::Matrix3f K = BNLM::Matrix3f::Identity();
	K(0, 0) = fx * scaleFactor;
	K(1, 1) = fy * scaleFactor;
	K(0, 2) = cx * scaleFactor;
	K(1, 2) = cy * scaleFactor;

	return K;
}

void ComputeInitialExtrinsicsFromHomographiesAndIntrinsics(CameraCalibrationSolverSystem* camCalib, const Vector<BNLM::Matrix3f>& homographies) {
	ASSERT(camCalib->solverImages.count == homographies.count);

	BNLM::Matrix3f intrinsics = BNLM::Matrix3f::Identity();
	intrinsics(0, 0) = camCalib->fx;
	intrinsics(1, 1) = camCalib->fy;
	intrinsics(0, 2) = camCalib->cx;
	intrinsics(1, 2) = camCalib->cy;

	// We invert the intrinsics so we can multiply it out of the homography, and get just rotation + translation
	BNLM::Matrix3f invIntrinsics = BNLM::Matrix3f::Identity();
	invIntrinsics(0, 0) = 1.0f / intrinsics(0, 0);
	invIntrinsics(1, 1) = 1.0f / intrinsics(1, 1);
	invIntrinsics(0, 2) = -intrinsics(0, 2) * invIntrinsics(0, 0);
	invIntrinsics(1, 2) = -intrinsics(1, 2) * invIntrinsics(1, 1);

	BNS_VEC_FOR_I(camCalib->solverImages) {

		BNLM::Matrix3f H = invIntrinsics * homographies.data[i];

		//     |          |
		// H = | r1  r2 t |
		//     |          |
		BNLM::Vector3f r1 = H.column(0), r2 = H.column(1), t = H.column(2);

		float r1Norm = r1.Mag();
		float r2Norm = r2.Mag();

		float tNorm = (r1Norm + r2Norm) * 0.5f;
		r1 = r1 / r1Norm;
		r2 = r2 / r2Norm;
		t = t / tNorm;

		float overlap = BNLM::DotProduct(r1, r2);

		BNLM::Vector3f r3 = BNLM::CrossProduct(r1, r2).Normalized();
		r2 = BNLM::CrossProduct(r3, r1);

		BNLM::Matrix3f rotation;
		rotation.block<3, 1>(0, 0) = r1;
		rotation.block<3, 1>(0, 1) = r2;
		rotation.block<3, 1>(0, 2) = r3;

		camCalib->solverImages.data[i].rotation = ConvertRotationMatrixToEulerAngles(rotation);
		camCalib->solverImages.data[i].translation = t;

		//BNS_VEC_FOREACH(camCalib->solverImages.data[i].checkerboardPoints) {
		//	BNLM::Vector3f camSpace = rotation * BNLM::Vector3f(ptr->planePt.x(), ptr->planePt.y(), 0.0f) + t;
		//	BNLM::Vector2f reproPt = (intrinsics * camSpace).hnorm();
		//	BNLM::Vector2f diff = (reproPt - ptr->imagePt);
		//	printf("diff: %f\n", diff.SquareMag());
		//}
	}
}

BNLM::Vector2f GetImagePointForCameraSpacePoint(CameraCalibrationSolverSystem* camCalib, const BNLM::Vector3f camSpace) {
	float screenX = camSpace.x() / camSpace.z();
	float screenY = camSpace.y() / camSpace.z();

	float r2 = BNS_SQR(screenX) + BNS_SQR(screenY);

	// Re-distort
	// TODO: Tangential distortion as well
	screenX = screenX / (1.0f + camCalib->distK(0) * r2 + camCalib->distK(1) * BNS_SQR(r2));
	screenY = screenY / (1.0f + camCalib->distK(0) * r2 + camCalib->distK(1) * BNS_SQR(r2));

	BNLM::Vector2f pixelPt;
	pixelPt.x() = camCalib->fx * screenX + camCalib->cx;
	pixelPt.y() = camCalib->fy * screenY + camCalib->cy;

	return pixelPt;
}

float ComputeReprojectionErrorForSolverImage(CameraCalibrationSolverSystem* camCalib, int imageIndex) {
	ASSERT(imageIndex < camCalib->solverImages.count);

	float totalErr = 0.0f;

	auto* calibImage = &camCalib->solverImages.data[imageIndex];

	BNLM::Matrix3f rotationMat = BNLM::ConvertEulerAnglesToRotationMatrix(calibImage->rotation);

	BNS_VEC_FOREACH(calibImage->checkerboardPoints) {
		BNLM::Vector3f cameraSpace = rotationMat * BNLM::Vector3f(ptr->planePt.x(), ptr->planePt.y(), 0.0f) + calibImage->translation;
		BNLM::Vector2f pixelReprojection = GetImagePointForCameraSpacePoint(camCalib, cameraSpace);
		totalErr += (ptr->imagePt - pixelReprojection).SquareMag();
	}

	return totalErr;
}

float ComputeReprojectionErrorForSolverSystem(CameraCalibrationSolverSystem* camCalib) {
	float totalErr = 0.0f;

	BNS_VEC_FOR_I(camCalib->solverImages) {
		totalErr += ComputeReprojectionErrorForSolverImage(camCalib, i);
	}

	return totalErr;
}

// 
void OptimiseExtrinsicsAndIntrinsicsAndDistrotionForCameras(CameraCalibrationSolverSystem* camCalib) {
	// TODO: distP as well
	const int intrinsicsParamCount = 6;

	const int parameterCount = intrinsicsParamCount + 6 * camCalib->solverImages.count;

	Vector<float*> manipulableFloats;
	manipulableFloats.EnsureCapacity(parameterCount);

	manipulableFloats.PushBack(&camCalib->fx);
	manipulableFloats.PushBack(&camCalib->fy);
	manipulableFloats.PushBack(&camCalib->cx);
	manipulableFloats.PushBack(&camCalib->cy);
	manipulableFloats.PushBack(&camCalib->distK.x());
	manipulableFloats.PushBack(&camCalib->distK.y());


	BNS_VEC_FOREACH(camCalib->solverImages) {
		BNS_FOR_I(3) {
			manipulableFloats.PushBack(&ptr->rotation(i));
			manipulableFloats.PushBack(&ptr->translation(i));
		}
	}

	ASSERT(manipulableFloats.count == parameterCount);

	Vector<float> errorDifferences;
	errorDifferences.EnsureCapacity(parameterCount);

	float stepSize = 512.0f;
	const float minStepSize = 0.0000001f;

	int iter = 0;
	const int maxIters = 1000;

	const float calcStep = 0.0001f;

	float previousError = ComputeReprojectionErrorForSolverSystem(camCalib);
	printf("Starting reprojection error: %f\n", previousError);

	auto calcErrorForParameter = [=](int paramIdx) {
		if (paramIdx < intrinsicsParamCount) {
			return ComputeReprojectionErrorForSolverSystem(camCalib);
		}
		else {
			const int imageIndex = (paramIdx - 6) / 6;
			return ComputeReprojectionErrorForSolverImage(camCalib, imageIndex);
		}
	};

	Vector<float> previousVals;
	previousVals.Resize(parameterCount);

	while (stepSize > minStepSize && iter < maxIters) {
		errorDifferences.Clear();

		BNS_FOR_I(parameterCount) {
			float prevVal = *manipulableFloats.data[i];

			*manipulableFloats.data[i] = prevVal - calcStep;
			float newErr0 = calcErrorForParameter(i);

			*manipulableFloats.data[i] = prevVal + calcStep;
			float newErr1 = calcErrorForParameter(i);

			// Compute partial derivate of error function w.r.t. this parameter
			errorDifferences.PushBack((newErr1 - newErr0) / (calcStep * 2.0f));

			*manipulableFloats.data[i] = prevVal;
		}

		float totalMagnitude = 0.0f;
		BNS_VEC_FOLDR(totalMagnitude, errorDifferences, 0.0f, acc = acc + BNS_SQR(item));

		if (totalMagnitude == 0.0f) {
			printf("totalMagnitude is zero, exiting!\n");
			break;
		}

		// Scale the error differences to a unit-length vector
		float s = 1.0f / sqrt(totalMagnitude);
		BNS_VEC_FOREACH(errorDifferences) {
			*ptr *= s;
		}

		float currentError = 0.0f;

		do {
			// Save the parameters old value (in case it needs to be reset)
			BNS_FOR_I(parameterCount) {
				float prevVal = *manipulableFloats.data[i];
				previousVals.data[i] = prevVal;
				*manipulableFloats.data[i] = prevVal - errorDifferences.data[i] * stepSize;
			}

			currentError = ComputeReprojectionErrorForSolverSystem(camCalib);

			if (currentError < previousError) {
				previousError = currentError;
				break;
			}
			else {
				// Reset the parameters
				BNS_FOR_I(parameterCount) {
					*manipulableFloats.data[i] = previousVals.data[i];
				}
				stepSize = stepSize / 2.0f;
			}
		} while (stepSize > minStepSize && currentError > previousError);

		iter++;
	}

	printf("New reprojection error: %f\n", previousError);

	printf("Optimised values:\n");
	printf("  fx: %f:\n", camCalib->fx);
	printf("  fy: %f:\n", camCalib->fy);
	printf("  cx: %f:\n", camCalib->cx);
	printf("  cy: %f:\n", camCalib->cy);
	printf("  k1: %f:\n", camCalib->distK(0));
	printf("  k2: %f:\n", camCalib->distK(1));
}



