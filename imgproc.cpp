#include "BNImage.h"

#include "external/BNLM/core.h"

#include "external/BNLM/CppUtils/macros.h"
#include "external/BNLM/CppUtils/strings.h"

// TODO: Might want to add this to BNLM itself...
namespace BNLM {

using Vector2i = BNLM::Vector<int, 2>;

}

// TODO: Put this in vector.h?
#define BNS_VEC_DUMB_SORT(vec, pred) do {                   \
	BNS_FOR_I((vec).count) {                                \
		for (int j = i; j > 0; j--) {                       \
			auto l = (vec).data[j - 1];						\
			auto r = (vec).data[j ];						\
			if (!(pred)) {									\
				auto tmp = (vec).data[j];					\
				(vec).data[j] = (vec).data[j - 1];			\
				(vec).data[j - 1] = tmp;					\
			}												\
			else {											\
				break;										\
			}												\
		}													\
	}                                                       \
} while(0)

void ApplyGaussianFilterToGSImage(const BNImage<unsigned char>& img, BNImage<unsigned char>* outImg) {
	ASSERT(outImg->width == img.width);
	ASSERT(outImg->height == img.height);

	// TODO: Allocation
	BNImage<float> scratch(img.width, img.height);

	// TODO: Make this configurable
	const int kernelSize = 5;
	static_assert(kernelSize % 2 == 1, "Box size must be odd");
	float gaussianCoeffs[] = { 0.06136f, 0.24477f, 0.38774f, 0.24477f, 0.06136f };
	static_assert(BNS_ARRAY_COUNT(gaussianCoeffs) == kernelSize, "Gaussian kernel size must match");

	// TODO: What to do at corners + edges...
	for (int y = kernelSize / 2; y < img.height - kernelSize / 2; y++) {
		for (int x = kernelSize / 2; x < img.width - kernelSize / 2; x++) {
			float outVal = 0.0f;
			BNS_FOR_I(kernelSize) {
				// TODO(perf): abc
				outVal += (gaussianCoeffs[i] * ((float)*img.GetPixelPtr(x + i - kernelSize / 2, y)) / 255.0f);
			}

			// TODO(perf): abc
			*scratch.GetPixelPtr(x, y) = outVal;
		}
	}

	for (int y = kernelSize / 2; y < img.height - kernelSize / 2; y++) {
		for (int x = kernelSize / 2; x < img.width - kernelSize / 2; x++) {
			float outVal = 0.0f;
			BNS_FOR_I(kernelSize) {
				// TODO(perf): abc
				outVal += (gaussianCoeffs[i] * *scratch.GetPixelPtr(x, y + i - kernelSize / 2));
			}

			// TODO(perf): abc
			*outImg->GetPixelPtr(x, y) = (unsigned char)(outVal * 255.0f);
		}
	}
}

// TODO
void SobelResponseOnImage(const BNImage<unsigned char>& img, BNImage<float>* outGrad, BNImage<float>* outAngle) {
	const float sobelResponseX[3][3] = {
		{1, 0, -1},
		{2, 0, -2},
		{1, 0, -1},
	};

	const float sobelResponseY[3][3] = {
		{  1,  2,  1 },
		{  0,  0,  0 },
		{ -1, -2, -1 },
	};

	const int kernelSize = 3;
	const float sobelNorm = 3.4641f;

	const int w = img.width;
	const int h = img.height;

	// We can pre-supply an image buffer for the outputs, or just allocate one if none is given
	if (outGrad->width != w || outGrad->height != h) {
		*outGrad = BNImage<float>(w, h);
	}

	if (outAngle->width != w || outAngle->height != h) {
		*outAngle = BNImage<float>(w, h);
	}

	// Yeah yeah, perf and stuff...
	BNS_FOR_J(h) {
		BNS_FOR_I(w) {
			*outGrad->GetPixelPtr(i, j) = 0;
			*outAngle->GetPixelPtr(i, j) = 0;
		}
	}

	for (int y = kernelSize / 2; y < h - kernelSize / 2; y++) {
		for (int x = kernelSize / 2; x < w - kernelSize / 2; x++) {
			float responseX = 0.0f;
			float responseY = 0.0f;
			BNS_FOR_J(kernelSize) {
				BNS_FOR_I(kernelSize) {
					unsigned char pixelVal = *img.GetPixelPtr(x + i - kernelSize / 2, y + j - kernelSize / 2);
					responseX += (pixelVal / 255.0f * sobelResponseX[j][i]);
					responseY += (pixelVal / 255.0f * sobelResponseY[j][i]);
				}
			}

			responseX /= sobelNorm;
			responseY /= sobelNorm;

			*outGrad->GetPixelPtr(x, y) = responseX * responseX + responseY * responseY;
			*outAngle->GetPixelPtr(x, y) = atan2(responseY, responseX);
		}
	}
}

void SobelResponseNonMaxFilter(BNImage<float> gradImg, BNImage<float> angleImg) {

	const int kernelSize = 3;

	const int w = gradImg.width;
	const int h = gradImg.height;

	for (int y = kernelSize / 2; y < h - kernelSize / 2; y++) {
		for (int x = kernelSize / 2; x < w - kernelSize / 2; x++) {
			float grad = *gradImg.GetPixelPtr(x,y);
			float angle = *angleImg.GetPixelPtr(x,y) * BNS_RAD2DEG;
			if (angle < 0) { angle += 360.0f; }
			if (angle >= 180.0f) { angle -= 180.0f; } // We only care about symmetric direction about origin

			int offsets[2] = {};
			if (angle <= 22.5f) { offsets[0] = -1; offsets[1] = 1; }
			else if (angle <= 67.5f) { offsets[0] = -w - 1; offsets[1] = w + 1; }
			else if (angle <= 112.5f) { offsets[0] = -w; offsets[1] = w; }
			else if (angle <= 157.5f) { offsets[0] = -w + 1; offsets[1] = w - 1; }
			else if (angle <= 180.0f) { offsets[0] = -1; offsets[1] = 1; } // Also horizontal gradient
			else {
				ASSERT(false);
			}

			BNS_FOR_I(2) {
				float otherGrad = gradImg.GetPixelPtr(x, y)[offsets[i]];
				if (otherGrad > grad) {
					*gradImg.GetPixelPtr(x, y) = 0.0f;
					break;
				}
			}
		}
	}
}

void MaskOutSobelResponseInNonSelectedAreas(BNImage<float> sobelGrad, const char* filename){
	FILE* pointsFile = fopen(StringStackBuffer<256>("%s.txt", filename).buffer, "rb");

	if (pointsFile == NULL) {
		printf("WARN: Could not open ('%s.txt')\n", filename);
	}

	if (pointsFile != NULL) {
		// NOTE: We do assume that the points are in order (i.e. no diagonals),
		// but we don't assume clockwise vs anti-clockwise
		BNLM::Vector2f points[4];

		BNS_ARRAY_FOR_J(points) {
			fscanf(pointsFile, "%f %f", &points[j](0), &points[j](1));
		}

		fclose(pointsFile);

		// TODO: Filtering...

		BNLM::Vector2f centroid = BNLM::Vector2f::Zero();
		BNLM::Vector2f minBounds = BNLM::Vector2f(FLT_MAX, FLT_MAX);
		BNLM::Vector2f maxBounds = BNLM::Vector2f(-FLT_MAX, -FLT_MAX);
		BNS_ARRAY_FOR_J(points) {
			centroid += points[j];
			minBounds.x() = BNS_MIN(minBounds.x(), points[j].x());
			maxBounds.x() = BNS_MAX(maxBounds.x(), points[j].x());
			minBounds.y() = BNS_MIN(minBounds.y(), points[j].y());
			maxBounds.y() = BNS_MAX(maxBounds.y(), points[j].y());
		}

		centroid /= 4;

		for (int y = 0; y < sobelGrad.height; y++) {
			if (y < minBounds.y() || y > maxBounds.y()) {
				for (int x = 0; x < sobelGrad.width; x++) {
					*sobelGrad.GetPixelPtr(x, y) = 0;
				}
				continue;
			}

			for (int x = 0; x < sobelGrad.width; x++) {
				if (x < minBounds.x() || x > maxBounds.x()) {
					*sobelGrad.GetPixelPtr(x, y) = 0;
					continue;
				}

				bool isInPoly = true;
				BNS_FOR_J(4) {
					int v0 = j, v1 = (j + 1) % 4;
					BNLM::Vector2f edge = points[v1] - points[v0];
					BNLM::Vector2f test1 = centroid - points[v0];
					BNLM::Vector2f test2 = BNLM::Vector2f(x, y) - points[v0];
					float sign1 = BNLM::CrossProduct(edge, test1);
					float sign2 = BNLM::CrossProduct(edge, test2);

					if (sign1 * sign2 < 0) {
						isInPoly = false;
						break;
					}
				}

				if (!isInPoly) {
					*sobelGrad.GetPixelPtr(x, y) = 0;
				}
			}
		}
	}
}

void HoughTransformAfterSobel(const BNImage<float>& gradImg, const BNImage<float>& angleImg, int thetaResolutionPerDegree, BNImage<short>* outVoting) {
	const int w = gradImg.width;
	const int h = gradImg.height;
	
	// Compute max possible bounds for rho (resolution is 1 pixel)
	const int rhoCount = (int)(sqrtf(w * w + h * h) + 1.0f) * 3;

	// Theta can have different resolutions if we want, recommend 3-5 per degree for now?
	int thetaCount = 180 * thetaResolutionPerDegree;

	if (outVoting->width != thetaCount || outVoting->height != rhoCount) {
		*outVoting = BNImage<short>(thetaCount, rhoCount);
	}

	BNS_FOR_J(rhoCount) {
		BNS_FOR_I(thetaCount) {
			*outVoting->GetPixelPtr(i, j) = 0;
		}
	}

	const int border = 2;

	const int thetaSearchSizeDegrees = 10;

	float gradientThreshold = 0.1f;

	for (int j = border; j < h - border; j++) {
		for (int i = border; i < w - border; i++) {
			float grad = *gradImg.GetPixelPtr(i, j);

			if (grad >= gradientThreshold) {

				float angle = *angleImg.GetPixelPtr(i, j);
				angle = angle * BNS_RAD2DEG;
				if (angle <  -90) { angle += 180.0f; }
				if (angle >= 90) { angle -= 180.0f; }

				int angleMin = (int)(angle - thetaSearchSizeDegrees);
				int angleMax = (int)(angle + thetaSearchSizeDegrees);

				int x = i, y = j;
				for (int theta = angleMin * thetaResolutionPerDegree; theta < angleMax * thetaResolutionPerDegree; theta++) {

					int realTheta = theta;

					// Get theta within [-90, 90) degree range
					if (realTheta < -90 * thetaResolutionPerDegree) {
						realTheta += 90 * thetaResolutionPerDegree;
					}
					if (realTheta >= 90 * thetaResolutionPerDegree) {
						realTheta -= 90 * thetaResolutionPerDegree;
					}
					
					float thetaRad = (BNS_DEG2RAD * realTheta) / thetaResolutionPerDegree;
					float rho = x * cosf(thetaRad) + y * sinf(thetaRad);
					int votingX = realTheta + 90 * thetaResolutionPerDegree;
					int votingY = ((int)rho + rhoCount / 2);
					(*outVoting->GetPixelPtr(votingX, votingY))++;
				}
			}
		}
	}
}

struct HoughLocalMaximum {
	//int x; // theta
	//int y; // rho
	float thetaDegrees;
	float rho;
	int score = 0;
};

void FindLocalMaximaInHoughTransform(const BNImage<short>& voting, int thetaResolutionPerDegree, Vector<HoughLocalMaximum>* outLocalMaxima) {
	const int thetaCount = voting.width;
	const int rhoCount = voting.height;

	const int localNeighboorhoodSizeRho = 10;
	const int localNeighboorhoodSizeTheta = 4 * thetaResolutionPerDegree;
	
	const int minimumVoteCount = 60;

	BNS_FOR_I(rhoCount) {
		BNS_FOR_J(thetaCount) {
			bool isLocalMax = true;
			int currentScore = *voting.GetPixelPtr(j, i);
			if (currentScore < minimumVoteCount) {
				continue;
			}

			// TODO: Rho can be sign-flipped here as well...
			for (int jOff = -localNeighboorhoodSizeTheta; jOff <= localNeighboorhoodSizeTheta; jOff++) {
				for (int iOff = -localNeighboorhoodSizeRho; iOff <= localNeighboorhoodSizeRho; iOff++) {
					if (jOff != 0 || iOff != 0) {
						int newTheta = j + jOff;
						int newRho = i + iOff;

						// TODO:::::::
						if (newRho < 0 || newRho >= rhoCount) {
							continue;
						}

						if (newTheta >= thetaCount) {
							newTheta -= thetaCount;
							newRho = rhoCount - newRho;
						}
						else if (newTheta < 0) {
							newTheta += thetaCount;
							newRho = rhoCount - newRho;
						}

						int otherScore = *voting.GetPixelPtr(newTheta, newRho);
						if (currentScore < otherScore) {
							isLocalMax = false;
							break;
						}
						else if (currentScore == otherScore) {
							// If we have a pixel w/ a higher score, we suppress ourselves
							// But in the case of two lines w/ equal score, we need to suppress one but not the other
							// So here we just pick the one with a higher theta/rho combo.
							// This is okay, since this is just an approximation/first pass.
							if (jOff < 0 || (jOff == 0 && iOff < 0)) {
								isLocalMax = false;
								break;
							}
						}
					}
				}

				// Early break if we've already found a suppressor for this pixel
				if (!isLocalMax) {
					break;
				}
			}

			if (isLocalMax) {
				HoughLocalMaximum localMax;

				float thetaDegrees = ((float)j - 90 * thetaResolutionPerDegree) / thetaResolutionPerDegree;
				float rho = i - (voting.height / 2);

				localMax.thetaDegrees = thetaDegrees;
				localMax.rho = rho;
				localMax.score = currentScore;
				outLocalMaxima->PushBack(localMax);
			}
		}
	}

	BNS_VEC_DUMB_SORT(*outLocalMaxima, l.score > r.score);
}

// Split local maxima in Hough Space into vertical lines and horizontal lines
// Also sort the lines by distance from the origin
void FilterAndSortVerticalAndHorizontalHoughBuckets(const Vector<HoughLocalMaximum>& localMaxima,
													Vector<HoughLocalMaximum>* outVertical,
													Vector<HoughLocalMaximum>* outHorizontal) {
	
	outVertical->Clear();
	outHorizontal->Clear();

	float avgAbsTheta = 0.0f;
	BNS_VEC_FOLDR(avgAbsTheta, localMaxima, 0.0f, acc + BNS_ABS(item.thetaDegrees));

	avgAbsTheta /= localMaxima.count;

	// The exact distinction b/w horizontal and vertical isn't too important,
	// just that w/in groups the lines are parallel and b/w the groups they're roughly perpendicular
	BNS_VEC_FOREACH(localMaxima) {
		if (BNS_ABS(ptr->thetaDegrees) > avgAbsTheta) {
			outHorizontal->PushBack(*ptr);
		}
		else {
			outVertical->PushBack(*ptr);
		}
	}

	BNS_VEC_DUMB_SORT(*outHorizontal, BNS_ABS(l.rho) < BNS_ABS(r.rho));
	BNS_VEC_DUMB_SORT(*outVertical, BNS_ABS(l.rho) < BNS_ABS(r.rho));
}

struct CheckerboardCorner {
	BNLM::Vector2f imagePt;
	BNLM::Vector2f planePt;
};

void FindInitialCheckerboardCorners(const Vector<HoughLocalMaximum>& verticalLines,
									const Vector<HoughLocalMaximum>& horizontalLines,
									Vector<CheckerboardCorner>* outCorners) {
	//

	outCorners->Clear();
	outCorners->EnsureCapacity(verticalLines.count * horizontalLines.count);

	// Line intersection:
	// I swear, I just had this written down somewhere
	// Please I hope it works
	// a1x + b1y = c1
	// a2x + b2y = c2
	// a1/a2(a2x + b2y) = a1/a2*c2
	// a1x + a1/a2*b2y = a1/a2*c2
	// a1/a2*b2y - b1y = a1/a2*c2 - c1
	// y = (a1/a2*c2 - c1) / (a1/a2 * b2  - b1)
	// x = (c1 - b1y) / a1

	BNS_VEC_FOR_I(horizontalLines) {

		HoughLocalMaximum horizontalLine = horizontalLines.data[i];
		float thetaDegrees1 = horizontalLine.thetaDegrees;
		float c1 = horizontalLine.rho;
		float theta1 = thetaDegrees1 * BNS_DEG2RAD;

		float a1 = cosf(theta1), b1 = sinf(theta1);

		BNS_VEC_FOR_J(verticalLines) {
			HoughLocalMaximum verticalLine = verticalLines.data[j];
			int thetaDegrees2 = verticalLine.thetaDegrees;
			float c2 = verticalLine.rho;
			float theta2 = thetaDegrees2 * BNS_DEG2RAD;

			float a2 = cosf(theta2), b2 = sinf(theta2);

			float imageY = (a1 / a2 * c2 - c1) / (a1 / a2 * b2 - b1);
			float imageX = (c1 - b1 * imageY) / a1;

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

	float val00 = *img.GetPixelPtr(x0,     y0);
	float val01 = *img.GetPixelPtr(x0,     y0 + 1);
	float val10 = *img.GetPixelPtr(x0 + 1, y0);
	float val11 = *img.GetPixelPtr(x0 + 1, y0 + 1);

	float val0 = LerpFloat(val00, val01, fracY);
	float val1 = LerpFloat(val10, val11, fracY);
	return LerpFloat(val0, val1, fracX) / 255.0f;
}

// We give an initial estimate for the checkerboard corner, and now look for a better one
// in a local radius (since the initial guess didn't take distortion into account)
// TODO: Like...any bounds checking?
void RefineCheckerboardCornerPositionsInImage(BNImage<unsigned char> img, const int searchSize, Vector<CheckerboardCorner>* corners) {

	const int searchSquareSize = searchSize * 2 + 1;

	BNImage<float> gradMask(searchSquareSize, searchSquareSize);

	BNS_FOR_J(searchSquareSize) {
		float yOff = j - searchSize;
		float yMask = exp(-BNS_SQR(yOff));
		BNS_FOR_I(searchSquareSize) {
			float xOff = i - searchSize;
			float xMask = exp(-BNS_SQR(xOff));
			*gradMask.GetPixelPtr(i,j) = xMask * yMask;
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

			BNLM::Vector2f guessMovement (newX, newY);
			float err = guessMovement.SquareMag();

			if (err <= 0.0000001f) {
				break;
			}

			currentGuess = currentGuess + guessMovement;

			iter++;
		}

		float totalChangeSqr = (ptr->imagePt - currentGuess).SquareMag();

		// TODO: Parameterise?
		{//if (totalChangeSqr < 100.0f) {
			ptr->imagePt = currentGuess;
		}
	}
}

BNLM::Matrix3f ComputeHomographyFromCheckerboardCorners(const Vector<CheckerboardCorner>& corners, const float scaleFactor) {
	int rowCount = 2 * corners.count;

	BNLM::MatrixXf solverSystem(rowCount, 9);
	solverSystem.ZeroOut();

	BNS_VEC_FOR_I(corners) {
		float u1 = corners.data[i].imagePt.x() / scaleFactor, v1 = corners.data[i].imagePt.y() / scaleFactor;
		float u2 = corners.data[i].planePt.x(), v2 = corners.data[i].planePt.y();

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

void DrawLineOnRGBImage(BNImage<unsigned char, 3> image, int x0, int y0, int x1, int y1, BN_RGB col) {
	if (x0 == x1) {
		for (int y = BNS_MIN(y0, y1); y < BNS_MAX(y0, y1); y++) {
			unsigned char* pixel = image.GetPixelPtr(x0, y);
			BNS_FOR_NAME(k, 3) {
				pixel[k] = col.rgb[k];
			}
		}
	}

	int dx =  BNS_ABS(x1 - x0), sx = (x0 < x1) ? 1 : -1;
	int dy = -BNS_ABS(y1 - y0), sy = (y0 < y1) ? 1 : -1;
	int err = dx + dy;
	while (true) {
		unsigned char* pixel = image.GetPixelPtr(x0, y0);
		BNS_FOR_NAME(k, 3) {
			pixel[k] = col.rgb[k];
		}

		if (x0 == x1 && y0 == y1) { break; }

		int e2 = 2 * err;
		if (e2 >= dy) { err += dy; x0 += sx; }
		if (e2 <= dx) { err += dx; y0 += sy; }

		if (x0 < 0 || x0 >= image.width || y0 < 0 || y0 >= image.height) { break; }
	}
}

// TODO: Draw text on images?


// TODO: FAST corners + ORB features

