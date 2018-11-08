#include "BNImage.h"

#define STB_IMAGE_IMPLEMENTATION
#include "external/stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "external/stb_image_write.h"

BNImage<unsigned char> LoadGSImageFromFile(const char* filename) {
	int x, y, n;
	unsigned char* data = stbi_load(filename, &x, &y, &n, 1);

	if (data == nullptr) {
		printf("Warning: Could not open file '%s'\n", filename);
		return BNImage<unsigned char>();
	}

	BNImage<unsigned char> dataWrapper(x, y, data);

	// TODO: Avoid this copy somehow?
	BNImage<unsigned char> dataCpy = dataWrapper.GetDeepCopy();

	free(data);

	return dataCpy;
}

BNImage<unsigned char, 3> LoadRGBImageFromFile(const char* filename) {
	int x, y, n;
	unsigned char* data = stbi_load(filename, &x, &y, &n, 3);

	if (data == nullptr) {
		printf("Warning: Could not open file '%s'\n", filename);
		return BNImage<unsigned char, 3>();
	}

	BNImage<unsigned char, 3> dataWrapper(x, y, data);

	// TODO: Avoid this copy somehow?
	BNImage<unsigned char, 3> dataCpy = dataWrapper.GetDeepCopy();

	free(data);

	return dataCpy;
}

void SaveGSImageToPNGFile(const char* filename, BNImage<unsigned char, 1> img) {
	//
	stbi_write_png(filename, img.width, img.height, 1, img.imageStart, img.strideInBytes);
}

void SaveRGBImageToPNGFile(const char* filename, BNImage<unsigned char, 3> img) {
	//
	stbi_write_png(filename, img.width, img.height, 3, img.imageStart, img.strideInBytes);
}

BNImage<int> LoadARGBImageFromFile(const char* filename) {
	int x, y, n;
	unsigned char* data = stbi_load(filename, &x, &y, &n, 4);

	BNImage<int> imgRaw(x, y, data);

	BNImage<int> imgCpy = imgRaw.GetDeepCopy();

	stbi_image_free(data);

	return imgCpy;
}

