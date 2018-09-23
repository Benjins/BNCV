#include "BNImage.h"

#define STB_IMAGE_IMPLEMENTATION
#include "external/stb_image.h"

BNImage<int> LoadARGBImageFromFile(const char* filename) {
	int x, y, n;
	unsigned char* data = stbi_load(filename, &x, &y, &n, 4);

	BNImage<int> imgRaw(x, y, data);

	BNImage<int> imgCpy = imgRaw.GetDeepCopy();

	stbi_image_free(data);

	return imgCpy;
}

