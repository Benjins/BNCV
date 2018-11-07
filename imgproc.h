#ifndef IMGPROC_H_
#define IMGPROC_H_

#pragma once

#include "BNImage.h"

// TODO: Colour versions too
void ApplyGaussianFilterToGSImage(const BNImage<unsigned char>& img, BNImage<unsigned char>* outImg);

struct HoughLocalMaximum {
	//int x; // theta
	//int y; // rho
	float thetaDegrees;
	float rho;
	int score = 0;
};

struct CheckerboardCorner {
	BNLM::Vector2f imagePt;
	BNLM::Vector2f planePt;
};

void SobelResponseOnImage(const BNImage<unsigned char>& img, BNImage<float>* outGrad, BNImage<float>* outAngle);

void SobelResponseNonMaxFilter(BNImage<float> gradImg, BNImage<float> angleImg);

void HoughTransformAfterSobel(const BNImage<float>& gradImg, const BNImage<float>& angleImg, int thetaResolutionPerDegree, BNImage<short>* outVoting);

void FindLocalMaximaInHoughTransform(const BNImage<short>& voting, int thetaResolutionPerDegree, Vector<HoughLocalMaximum>* outLocalMaxima,
									 const int localNeighboorhoodSizeRho = 10, const int minimumVoteCount = 60);

void FilterAndSortVerticalAndHorizontalHoughBuckets(const Vector<HoughLocalMaximum>& localMaxima,
													Vector<HoughLocalMaximum>* outVertical,
													Vector<HoughLocalMaximum>* outHorizontal);


#endif

