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
	auto img1 = LoadGSImageFromFile("C:/Users/Benji/CVDatasets/android_lg_g5/still_1540083470/image_00010_Y.png");
	auto img2 = LoadGSImageFromFile("C:/Users/Benji/CVDatasets/android_lg_g5/still_1540083470/image_00012_Y.png");

	const int fastThreshold = 20;

	Vector<BNFastKeyPoint> kpts1;
	Vector<BNORBDescriptor> desc1;
	FindORBFeaturePointsInGSImage(img1, fastThreshold, &kpts1, &desc1);

	auto img1rgb = ConvertGSImageToRGB(img1);
	
	BNS_VEC_FOREACH(kpts1) {
		int x = (int)ptr->imagePoint.x();
		int y = (int)ptr->imagePoint.y();

		auto subImg = img1.GetSubImage(x - 16, y - 16, 32, 32);

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

	BNLM::Matrix3f fundamental;
	CalculateFundamentalMatrixUsing8PointAlgorithm(kpts1, kpts2, matches, &fundamental);

	BNImage<unsigned char, 3> flowImg = img1rgb.GetDeepCopy();

	BNS_VEC_FOREACH_NAME(matches, match) {
		BNLM::Vector2f kpt1 = kpts1.data[match->srcIdx].imagePoint;
		BNLM::Vector2f kpt2 = kpts2.data[match->targetIdx].imagePoint;

		float err = BNS_ABS(BNLM::DotProduct(kpt2.homo(), fundamental * kpt1.homo()));

		float normErr = BNS_MIN(err * 5.0f, 1.0f);

		BN_RGB green = { 40, (unsigned char)((1.0f - normErr) * 250), (unsigned char)(normErr * 250) };

		int x0 = (int)kpt1.x();
		int y0 = (int)kpt1.y();

		int x1 = (int)kpt2.x() + img1.width;
		int y1 = (int)kpt2.y();

		DrawLineOnRGBImage(matchImg, x0, y0, x1, y1, green);
		DrawLineOnRGBImage(flowImg, x0, y0, x1 - img1.width, y1, green);

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
