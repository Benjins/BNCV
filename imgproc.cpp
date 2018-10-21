#include "BNImage.h"

#include "external/BNLM/core.h"

// TODO: Might want to add this to BNLM itself...
namespace BNLM {

using Vector2i = BNLM::Vector<int, 2>;

}

void ApplyGaussianFilterToImage(const BNImage<unsigned char>& img, BNImage<unsigned char>* outImg) {
	//BNS_PROFILE_SCOPE(__FUNCTION__);

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

void DrawLineOnRGBImage(BNImage<unsigned char, 3>* img, int x0, int y0, int x1, int y1, BN_RGB col) {

}

// TODO: Draw text on images?


// TODO: FAST corners + ORB features

