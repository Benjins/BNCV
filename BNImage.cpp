#include "BNImage.h"

#define STB_IMAGE_IMPLEMENTATION
#include "external/stb_image.h"

BNImage<unsigned char> LoadGSImageFromFile(const char* filename) {
	int x, y, n;
	unsigned char* data = stbi_load(filename, &x, &y, &n, 1);

	BNImage<unsigned char> dataWrapper(x, y, data);

	// TODO: Avoid this copy somehow?
	BNImage<unsigned char> dataCpy = dataWrapper.GetDeepCopy();

	free(data);

	return dataCpy;
}

