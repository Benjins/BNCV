#include "BNImage.h"

#include "external/BNLM/core.h"

#include "external/BNLM/CppUtils/macros.h"

// TODO: Might want to add this to BNLM itself...
namespace BNLM {

using Vector2i = BNLM::Vector<int, 2>;

}

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

}

void HoughTransformAfterSobel(const BNImage<float>& grad, const BNImage<float>& angle, int thetaCount, int rhoCount, BNImage<short>* outVoting) {

}

void FindLocalMaximaInHoughTransform(const BNImage<short>& voting, Vector<BNLM::Vector2i>* outLocalMaxima) {
	
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

