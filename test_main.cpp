#include <stdio.h>

#include "logging.h"

#include "BNImage.h"

#include "external/BNLM/CppUtils/vector.h"
#include "external/BNLM/CppUtils/assert.cpp"

int main() {
	printf("Yo.\n");

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
	
	return 0;
}