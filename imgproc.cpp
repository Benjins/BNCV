#include "BNImage.h"

#include "external/BNLM/core.h"

#include "external/BNLM/CppUtils/macros.h"

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

void HoughTransformAfterSobel(const BNImage<float>& gradImg, const BNImage<float>& angleImg, int thetaResolutionPerDegree, BNImage<short>* outVoting) {
	const int w = gradImg.width;
	const int h = gradImg.height;
	
	// Compute max possible bounds for rho (resolution is 1 pixel)
	const int rhoCount = (int)(sqrtf(w * w + h * h) + 1.0f) * 2;

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

	const int thetaSearchSize = 8;

	float gradientThreshold = 0.01f;

	for (int j = border; j < h - border; j++) {
		for (int i = border; i < w - border; i++) {
			float grad = *gradImg.GetPixelPtr(i, j);

			if (grad >= gradientThreshold) {

				float angle = *angleImg.GetPixelPtr(i, j);
				angle = angle * BNS_RAD2DEG;
				if (angle <  -90) { angle += 180.0f; }
				if (angle >= 90) { angle -= 180.0f; }

				int angleMin = (int)(angle - thetaSearchSize);
				int angleMax = (int)(angle + thetaSearchSize);
				angleMin = BNS_MAX(angleMin, -90);
				angleMax = BNS_MIN(angleMax,  90);

				int x = i, y = j;
				for (int theta = angleMin * thetaResolutionPerDegree; theta < angleMax * thetaResolutionPerDegree; theta++) {
					float thetaRad = (BNS_DEG2RAD * theta) / thetaResolutionPerDegree;
					float rho = x * cosf(thetaRad) + y * sinf(thetaRad);
					int votingX = theta + 90 * thetaResolutionPerDegree;
					int votingY = ((int)rho + rhoCount / 2);
					(*outVoting->GetPixelPtr(votingX, votingY))++;
				}
			}
		}
	}
}

struct HoughLocalMaximum {
	int x; // theta
	int y; // rho
	int score = 0;
};

void FindLocalMaximaInHoughTransform(const BNImage<short>& voting, int thetaResolutionPerDegree, Vector<HoughLocalMaximum>* outLocalMaxima) {
	const int thetaCount = voting.width;
	const int rhoCount = voting.height;

	const int localNeighboorhoodSizeRho = 2;
	const int localNeighboorhoodSizeTheta = 2 * thetaResolutionPerDegree;
	
	const int minimumVoteCount = 80;

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
						
						if (j == 722 && i == 1142) {
							int xc = 0;
							(void)xc;

							//if ()
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
						if (currentScore <= (otherScore + 2)) {
							isLocalMax = false;
						}
					}
				}
			}

			if (isLocalMax) {
				HoughLocalMaximum localMax;
				localMax.x = j;
				localMax.y = i;
				localMax.score = currentScore;
				outLocalMaxima->PushBack(localMax);
			}
		}
	}

	BNS_VEC_DUMB_SORT(*outLocalMaxima, l.score > r.score);
}

// TODO: Filter local maxima again if they're too close?

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

