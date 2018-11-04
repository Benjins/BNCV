#include <stdio.h>

#include "logging.h"

#include "BNImage.h"

#include "sfm_bootstrap.cpp"
#include "imgproc.cpp"
#include "feature_matching.cpp"
#include "features.cpp"
#include "BNImage.cpp"

#include "external/BNLM/CppUtils/vector.h"
#include "external/BNLM/CppUtils/assert.cpp"
#include "external/BNLM/CppUtils/strings.cpp"
#include "external/BNLM/CppUtils/bitset.cpp"

#include "external/BNLM/CppUtils/testing.h"	

BNImage<unsigned char, 3> ConvertGSImageToRGB(BNImage<unsigned char> img) {
	BNImage<unsigned char, 3> rgb(img.width, img.height);
	BNS_FOR_J(img.height) {
		BNS_FOR_I(img.width) {
			auto* srcPtr = img.GetPixelPtr(i, j);
			auto* dstPtr = rgb.GetPixelPtr(i, j);
			BNS_FOR_NAME(k, 3) { dstPtr[k] = *srcPtr; }
		}
	}

	return rgb;
}

#include "external/BNLM/test_main.cpp"

float GetRandomFloat01() {
	double num = rand() % RAND_MAX;
	const double invDenom = 1.0 / RAND_MAX;
	return (float)(num * invDenom);
}

float GetRandomFloatInRange(float min, float max) {
	float t = GetRandomFloat01();
	return min + t * (max - min);
}

CREATE_TEST_CASE("Eigen decomp stuff") {

	// TODO

	return 0;
}

CREATE_TEST_CASE("Camera calib") {
	const char* filenames[] = {
		"C:/Users/Benji/CVDatasets/android_lg_g5/still_1541265794/image_00000_Y.png",
		"C:/Users/Benji/CVDatasets/android_lg_g5/still_1541265794/image_00023_Y.png",
		"C:/Users/Benji/CVDatasets/android_lg_g5/still_1541265794/image_00042_Y.png",
		"C:/Users/Benji/CVDatasets/android_lg_g5/still_1541265794/image_00074_Y.png",
		"C:/Users/Benji/CVDatasets/android_lg_g5/still_1541265794/image_00084_Y.png",
	};

	BNS_FOR_I(3) {
		auto noSupp = LoadRGBImageFromFile(StringStackBuffer<256>("sobel_test_no_supp_%d.png", i).buffer);
		auto supp = LoadRGBImageFromFile(StringStackBuffer<256>("sobel_test_%d.png", i).buffer);

		int xc = 0;
		(void)xc;
	}

	BNS_ARRAY_FOR_I(filenames) {

		auto img1 = LoadGSImageFromFile(filenames[i]);

		BNImage<float> sobelGrad, sobelAngle;
		SobelResponseOnImage(img1, &sobelGrad, &sobelAngle);

		SobelResponseNonMaxFilter(sobelGrad, sobelAngle);

		const int thetaResolutionPerDegree = 4;

		BNImage<short> sobelHoughVoting;
		HoughTransformAfterSobel(sobelGrad, sobelAngle, thetaResolutionPerDegree, &sobelHoughVoting);

		Vector<HoughLocalMaximum> houghLocalMaxima;
		FindLocalMaximaInHoughTransform(sobelHoughVoting, thetaResolutionPerDegree, &houghLocalMaxima);

		{
			BNImage<unsigned char, 3> votingViz(sobelHoughVoting.width, sobelHoughVoting.height);

			BNS_FOR_NAME(y, sobelHoughVoting.height) {
				BNS_FOR_NAME(x, sobelHoughVoting.width) {
					BNS_FOR_NAME(k, 3) {
						votingViz.GetPixelPtr(x, y)[k] = *sobelHoughVoting.GetPixelPtr(x, y);
					}
				}
			}

			int expectedLineCount = 6;
			int localMaxCount = BNS_MIN(houghLocalMaxima.count, expectedLineCount);

			BNS_FOR_I(localMaxCount) {
				auto localMax = houghLocalMaxima.data[i];
				unsigned char* pixel = votingViz.GetPixelPtr(localMax.x, localMax.y);
				pixel[0] = 0;
				pixel[1] = 250;
				pixel[2] = 0;
			}

			auto rgbImg = ConvertGSImageToRGB(img1);

			BNS_FOR_I(localMaxCount) {
				auto localMax = houghLocalMaxima.data[i];
				int thetaDegrees = (localMax.x - 90 * thetaResolutionPerDegree) / thetaResolutionPerDegree;
				float rho = (localMax.y) - (votingViz.height / 2);
				float theta = thetaDegrees * BNS_DEG2RAD;
				float cosTheta = cosf(theta), sinTheta = sinf(theta);

				if (BNS_ABS(thetaDegrees) > 45) {
					BNS_FOR_I(rgbImg.width) {
						int x = i;
						int y = (int)((rho - x * cosTheta) / sinTheta);
						if (y >= 0 && y < rgbImg.height) {

							unsigned char* pixel = rgbImg.GetPixelPtr(x, y);

							pixel[0] = 220;
							pixel[1] = 0;
							pixel[2] = 50;
						}
					}
				} else {
					BNS_FOR_J(rgbImg.height) {
						int y = j;
						int x = (int)((rho - y * sinTheta) / cosTheta);
						if (x >= 0 && x < rgbImg.width) {
							BNS_FOR_NAME(b, 3) {
								unsigned char* pixel = rgbImg.GetPixelPtr(x, y);

								pixel[0] = 50;
								pixel[1] = 0;
								pixel[2] = 220;
							}
						}
					}
				}
			}

				SaveRGBImageToPNGFile(StringStackBuffer<256>("voting_viz_%d.png", i).buffer, votingViz);
				SaveRGBImageToPNGFile(StringStackBuffer<256>("line_viz_%d.png", i).buffer, rgbImg);

			int xc = 0;
			(void)xc;
		}

		BNImage<unsigned char, 3> testOut(img1.width, img1.height);

		BNS_FOR_J(img1.height) {
			BNS_FOR_I(img1.width) {
				float angle = *sobelAngle.GetPixelPtr(i, j);
				float mag = *sobelGrad.GetPixelPtr(i, j);
				float redness = sin(angle);
				float blueness = cos(angle);

				redness = BNS_ABS(redness);
				blueness = BNS_ABS(blueness);

				const float factor = 250.0f;

				unsigned char* bgr = testOut.GetPixelPtr(i, j);
				bgr[0] = (unsigned char)(blueness * mag * factor);
				bgr[1] = (unsigned char)(redness * mag * factor);
				bgr[2] = 0;
			}
		}

		SaveRGBImageToPNGFile(StringStackBuffer<256>("sobel_test_%d.png", i).buffer, testOut);
	}

	return 0;
}

CREATE_TEST_CASE("Singular vals fuzz") {
	const int testSeed = 100;
	printf("Seeding sing vals test w/ %d\n", testSeed);
	srand(testSeed);

	BNLM::Matrix3f mat = BNLM::Matrix3f::Identity();
	BNS_FOR_NAME(iter, 3) {

		//BNS_FOR_I(3) {
		//	BNS_FOR_J(3) {
		//		mat(i, j) = rand() % 8;// GetRandomFloatInRange(-3.0f, 3.0f);
		//	}
		//}

		double vals[] = {
			4.000000, 1.000000, 2.000000,
			5.000000, 7.000000, 8.000000,
			5.000000, 4.000000, 4.000000
		};

		BNS_FOR_I(9) {
			mat.data[i] = (float)vals[i];
		}

		mat(1, 1) += iter;

		printf("\n~~~~~~~~~~~~~~~~~~\n");

		BNLM::Matrix3f U, V;
		BNLM::Vector3f sigma;
		BNLM::SingularValueDecomposition(mat, &U, &sigma, &V);

		//printf("U matrix:\n");
		//printf("  %f %f %f\n", U(0, 0), U(0, 1), U(0, 2));
		//printf("  %f %f %f\n", U(1, 0), U(1, 1), U(1, 2));
		//printf("  %f %f %f\n", U(2, 0), U(2, 1), U(2, 2));
		//
		//printf("V matrix:\n");
		//printf("  %f %f %f\n", V(0, 0), V(0, 1), V(0, 2));
		//printf("  %f %f %f\n", V(1, 0), V(1, 1), V(1, 2));
		//printf("  %f %f %f\n", V(2, 0), V(2, 1), V(2, 2));

		//printf("singular vals: %f %f %f\n", sigma(0), sigma(1), sigma(2));

		BNLM::Matrix3f diag = BNLM::Matrix3f::Identity();
		BNS_FOR_I(3) {
			diag(i, i) = sigma(i);
		}
		BNLM::Matrix3f reconstMat = U * diag * V.transpose();

		printf("mat:\n");
		printf("  %f %f %f\n", mat(0, 0), mat(0, 1), mat(0, 2));
		printf("  %f %f %f\n", mat(1, 0), mat(1, 1), mat(1, 2));
		printf("  %f %f %f\n", mat(2, 0), mat(2, 1), mat(2, 2));

		printf("reconstMat:\n");
		printf("  %f %f %f\n", reconstMat(0, 0), reconstMat(0, 1), reconstMat(0, 2));
		printf("  %f %f %f\n", reconstMat(1, 0), reconstMat(1, 1), reconstMat(1, 2));
		printf("  %f %f %f\n", reconstMat(2, 0), reconstMat(2, 1), reconstMat(2, 2));

		BNS_FOR_I(3) {
			BNS_FOR_J(3) {
				//ASSERT_APPROX_WITH_EPS(reconstMat(i, j), mat(i, j), 0.01f);
			}
		}
	}

	ASSERT(false);

	return 0;
}

CREATE_TEST_CASE("Determinant of 3x3") {
	BNLM::Matrix3f mat;
	mat(0, 0) = 4;
	mat(0, 1) = -1;
	mat(0, 2) = 1;

	mat(1, 0) = 4;
	mat(1, 1) = 5;
	mat(1, 2) = 3;

	mat(2, 0) = -2;
	mat(2, 1) =  0;
	mat(2, 2) =  0;

	float det = mat.determinant();

	ASSERT(det == 16.0f);

	return 0;
}

CREATE_TEST_CASE("Basic FAST") {
	BNImage<unsigned char> img(50, 50);
	memset(img.baseData, 0, 50 * 50);
	*img.GetPixelPtr(30, 30) = 50;

	Vector<BNFastKeyPoint> kpts1;
	Vector<BNORBDescriptor> desc1;
	FindORBFeaturePointsInGSImage(img, 30, &kpts1, &desc1);

	ASSERT(kpts1.count == 1);
	ASSERT(kpts1.data[0].imagePoint.x() == 30);
	ASSERT(kpts1.data[0].imagePoint.y() == 30);

	return 0;
}

CREATE_TEST_CASE("Image FAST") {
	srand(20);

	auto img1 = LoadGSImageFromFile("C:/Users/Benji/CVDatasets/android_lg_g5/still_1540685717/image_00000_Y.png");
	auto img2 = LoadGSImageFromFile("C:/Users/Benji/CVDatasets/android_lg_g5/still_1540685717/image_00002_Y.png");

	const int fastThreshold = 20;

	Vector<BNFastKeyPoint> kpts1;
	Vector<BNORBDescriptor> desc1;
	FindORBFeaturePointsInGSImage(img1, fastThreshold, &kpts1, &desc1);

	auto img1rgb = ConvertGSImageToRGB(img1);
	
	BNS_VEC_FOREACH(kpts1) {
		int x = (int)ptr->imagePoint.x();
		int y = (int)ptr->imagePoint.y();

		auto subImg = img1.GetSubImage(x - 16, y - 16, 32, 32);
		auto subImg2 = img1.GetSubImage(x - 4, y - 4, 9,9);

		auto* pixel = img1rgb.GetPixelPtr(x, y);
		pixel[0] = 250;
		pixel[1] = 0;
		pixel[2] = 250;
	}

	Vector<BNFastKeyPoint> kpts2;
	Vector<BNORBDescriptor> desc2;
	FindORBFeaturePointsInGSImage(img2, fastThreshold, &kpts2, &desc2);

	auto img2rgb = ConvertGSImageToRGB(img2);

	BNS_VEC_FOREACH(kpts2) {
		int x = (int)ptr->imagePoint.x();
		int y = (int)ptr->imagePoint.y();

		auto* pixel = img2rgb.GetPixelPtr(x, y);
		pixel[0] = 250;
		pixel[1] = 0;
		pixel[2] = 250;
	}

	BN_RGB green = { 30, 250, 40 };

	//DrawLineOnRGBImage(img2rgb, 30, 30, 60, 400, green);

	Vector<BNFeatureMatch> matches;
	MatchFeaturesBasic(kpts1, desc1, kpts2, desc2, 70, 0.8f, &matches);

	BNImage<unsigned char, 3> matchImg(img1.width * 2, img1.height);
	img1rgb.DeepCopyTo(matchImg.GetSubImage(0, 0, img1.width, img1.height));
	img2rgb.DeepCopyTo(matchImg.GetSubImage(img1.width, 0, img2.width, img2.height));

	// Calibration from my LG G5 phone's camera, done offline
	// (Prooobably some play w/ the intrinsics buuuut w/e)
	/*
	  fx: 662.065979
	  fy: 661.942444
	  cx: 316.849243
	  cy: 256.200165
	  dist K: -0.032051 0.294078
	*/

	BNLM::Matrix3f intrinsics = BNLM::Matrix3f::Identity();
	intrinsics(0,0) = 662.065979f;
	intrinsics(1,1) = 661.942444f;
	intrinsics(0,2) = 316.849243f;
	intrinsics(1,2) = 256.200165f;
	BNLM::Vector2f distK;
	distK(0) = -0.032051f;
	distK(1) = 0.294078f;

	// - --- --- - - - - -- - - 

	BNLM::Matrix3f essential;
	CalculateEssentialMatrixUsing8PointAlgorithm(kpts1, kpts2, matches, intrinsics, distK, &essential);

	{
		BNLM::Matrix3f rotations[4];
		BNLM::Vector3f translations[4];
		DecomposeEssentialMatrixInto4MotionHypotheses(essential, rotations, translations);

		int xc = 0;
		(void)xc;

		// TODO: Triangulate I guess? Lol.

		{
			// --------------------------------------
			// TODO: Move to separate function?

			Vector<BNLM::Vector2f> points2DSrc;
			Vector<BNLM::Vector2f> points2DDst;

			points2DSrc.EnsureCapacity(matches.count);
			points2DDst.EnsureCapacity(matches.count);

			BNS_VEC_FOREACH(matches) {
				points2DSrc.PushBack(kpts1.data[ptr->srcIdx].imagePoint);
				points2DDst.PushBack(kpts2.data[ptr->targetIdx].imagePoint);
			}

			ConvertFromDistortedPixelCoordsToUndistortedNormalisedPoints(points2DSrc.data, points2DSrc.data, points2DSrc.count, intrinsics, distK);
			ConvertFromDistortedPixelCoordsToUndistortedNormalisedPoints(points2DDst.data, points2DDst.data, points2DDst.count, intrinsics, distK);

			Vector<bool> wasTri;
			Vector<BNLM::Vector3f> triPoints;

			wasTri.Resize(matches.count);
			triPoints.Resize(matches.count);

			for (int i = 0; i < 4; i++) {
				BNLM::Matrix4f w2c1 = BNLM::Matrix4f::Identity();
				BNLM::Matrix4f w2c2 = BNLM::Matrix4f::Identity();
				w2c2.block<3, 3>(0, 0) = rotations[i];
				w2c2.block<3, 1>(0, 3) = translations[i];

				int succCount1 = TriangulateNormalisedImagePoints(points2DSrc.data, points2DDst.data, matches.count,
																  w2c1, w2c2, intrinsics(0, 0), wasTri.data, triPoints.data);
			
				printf("Rotation hypothesis: (%f %f %f | %f %f %f | %f %f %f)\n",
											rotations[i](0, 0), rotations[i](0, 1), rotations[i](0, 2),
											rotations[i](1, 0), rotations[i](1, 1), rotations[i](1, 2),
											rotations[i](2, 0), rotations[i](2, 1), rotations[i](2, 2));

				printf("  succ count1: %d/%d\n", succCount1, matches.count);

				int succCount2 = TriangulateNormalisedImagePoints(points2DSrc.data, points2DDst.data, matches.count,
																  w2c2, w2c1, intrinsics(0, 0), wasTri.data, triPoints.data);

				printf("  succ count2: %d/%d\n", succCount2, matches.count);
			}
		}
	}

	BNImage<unsigned char, 3> flowImg = img1rgb.GetDeepCopy();

	BNS_VEC_FOREACH_NAME(matches, match) {
		BNLM::Vector2f kpt1 = kpts1.data[match->srcIdx].imagePoint;
		BNLM::Vector2f kpt2 = kpts2.data[match->targetIdx].imagePoint;

		BN_RGB green = { 40, 250, 40 };

		int x0 = (int)kpt1.x();
		int y0 = (int)kpt1.y();

		int x1 = (int)kpt2.x() + img1.width;
		int y1 = (int)kpt2.y();

		DrawLineOnRGBImage(matchImg, x0, y0, x1, y1, green);

		//printf("Err: %f\n", err);
	}

	SaveRGBImageToPNGFile("match_img.png", matchImg);
	SaveRGBImageToPNGFile("flow_img.png", flowImg);

	return 0;
}

CREATE_TEST_CASE("BNImage basic") {
	
	{
		BNImage<unsigned char, 2> img1;

		ASSERT(img1.baseData == nullptr);
		ASSERT(img1.imageStart == nullptr);
	}

	{
		BNImage<char, 2> img1(4, 4);

		ASSERT(img1.GetRefCount() == 1);
		
		BNImage<char, 2> img2 = img1;
		
		ASSERT(img1.GetRefCount() == 2);
		ASSERT(img2.GetRefCount() == 2);

		{
			BNImage<char, 2> img3 = img2;
			ASSERT(img1.GetRefCount() == 3);
		}

		ASSERT(img1.GetRefCount() == 2);
		ASSERT(img2.GetRefCount() == 2);

		{
			BNImage<char, 2> img3;
			img3 = img1;
			ASSERT(img1.GetRefCount() == 3);
			img3 = img2;
			ASSERT(img1.GetRefCount() == 3);
		}

		ASSERT(img1.GetRefCount() == 2);
		ASSERT(img2.GetRefCount() == 2);
	}

	{
		BNImage<unsigned char, 2> img1(4, 4);

		ASSERT(img1.width == 4);
		ASSERT(img1.height == 4);
		ASSERT(img1.strideInBytes == 8);
		ASSERT(img1.ownsMemory == true);
		ASSERT(img1.baseData != nullptr);
		ASSERT(img1.imageStart != nullptr);
		ASSERT(img1.imageStart == img1.baseData);

		unsigned char* ptr1 = img1.GetPixelPtr(0, 0);
		ASSERT(ptr1 == img1.imageStart);

		ptr1[0] = 0xFE;
		ptr1[1] = 0xFD;

		unsigned char* ptr2 = img1.GetPixelPtr(1, 0);
		ASSERT(ptr2 - ptr1 == 2);
		ptr2[0] = 0xFC;
		ptr2[1] = 0xFB;

		ASSERT(ptr1[0] == 0xFE);
		ASSERT(ptr1[1] == 0xFD);

		ASSERT(ptr2[0] == 0xFC);
		ASSERT(ptr2[1] == 0xFB);

		BNS_FOR_I(4) {
			BNS_FOR_J(4) {
				unsigned char* ptr = img1.GetPixelPtr(i, j);
				ptr[0] = i * 16 + j * 2;
				ptr[1] = i * 16 + j * 2 + 1;
			}
		}

		BNS_FOR_I(4) {
			BNS_FOR_J(4) {
				unsigned char* ptr = img1.GetPixelPtr(i, j);
				ASSERT(ptr[0] == i * 16 + j * 2);
				ASSERT(ptr[1] == i * 16 + j * 2 + 1);
			}
		}
	}

	{
		BNImage<unsigned char, 2> img1(4, 4);

		BNS_FOR_I(4) {
			BNS_FOR_J(4) {
				unsigned char* ptr = img1.GetPixelPtr(i, j);
				ptr[0] = i * 16 + j * 2;
				ptr[1] = i * 16 + j * 2 + 1;
			}
		}

		BNImage<unsigned char, 2> img2 = img1.GetDeepCopy();
		ASSERT(img2.width == 4);
		ASSERT(img2.height == 4);
		ASSERT(img2.baseData != img1.baseData);

		BNS_FOR_I(4) {
			BNS_FOR_J(4) {
				unsigned char* ptr = img2.GetPixelPtr(i, j);
				ASSERT(ptr[0] == i * 16 + j * 2);
				ASSERT(ptr[1] == i * 16 + j * 2 + 1);
			}
		}
	}

	{
		BNImage<unsigned char> img1(6, 6);

		memset(img1.GetPixelPtr(0, 0), 0, 6 * 6);

		{
			unsigned char* ptr = img1.GetPixelPtr(3, 3);
			ASSERT(*ptr == 0x00);
		}

		{
			BNImage<unsigned char> subImg = img1.GetSubImage(2, 2, 2, 2);
			ASSERT(subImg.width == 2);
			ASSERT(subImg.height == 2);
			ASSERT(subImg.strideInBytes == 6);

			unsigned char* ptr = subImg.GetPixelPtr(1, 1);
			*ptr = 0x7C;
		}

		unsigned char* ptr = img1.GetPixelPtr(3, 3);
		ASSERT(*ptr == 0x7C);
	}
	
	{
		BNImage<unsigned char> img1(2, 2);
		{
			unsigned char* ptr = img1.GetPixelPtr(0, 0);
			ptr[0] = 0x7E;
			ptr[1] = 0x2E;
			ptr[2] = 0x3E;
			ptr[3] = 0x4E;
		}
	
		BNImage<unsigned char> img2 = img1;

		BNImage<unsigned char> img3(2, 2);
		img2.DeepCopyTo(img3);

		{
			unsigned char* ptr = img3.GetPixelPtr(0, 0);
			ASSERT(ptr[0] == 0x7E);
			ASSERT(ptr[1] == 0x2E);
			ASSERT(ptr[2] == 0x3E);
			ASSERT(ptr[3] == 0x4E);
		}
	}

	{
		BNImage<unsigned char> img1(4, 4);
	}

	return 0;
}

#define CREATE_TEST_MAIN
#include "external/BNLM/CppUtils/testing.cpp"
