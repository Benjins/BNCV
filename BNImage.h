#ifndef BNIMAGE_H_
#define BNIMAGE_H_

#include <stdlib.h>

#include "external/BNLM/CppUtils/assert.h"

// Just for loop macros...blergh
#include "external/BNLM/CppUtils/vector.h"

template<typename _T, int _Channels = 1>
struct BNImage {

	enum {
		PixelBytes = sizeof(_T) * _Channels
	};

	void* baseData = nullptr;
	void* imageStart = nullptr;

	int width = 0;
	int height = 0;
	int strideInBytes = 0;

	bool ownsMemory = false;

	BNImage() { }

	~BNImage() {
		Release();
	}

	void Retain() const {
		if (ownsMemory && baseData != nullptr) {
			int* ref = ((int*)baseData) - 1;
			(*ref)++;
		}
	}

	void Release() {
		if (ownsMemory && baseData != nullptr) {
			int* ref = ((int*)baseData) - 1;
			(*ref)--;

			if (*ref == 0) {
				free(ref);

				baseData = nullptr;
				imageStart = nullptr;
			}
		}
	}

	int GetRefCount() const {
		if (ownsMemory && baseData != nullptr) {
			int* ref = ((int*)baseData) - 1;
			return *ref;
		}
		else {
			return 0;
		}
	}

	BNImage<_T, _Channels> GetSubImage(int x, int y, int w, int h) const {
		ASSERT(x >= 0 && x + w <= width);
		ASSERT(y >= 0 && y + h <= height);

		BNImage<_T, _Channels> subImg;
		subImg.baseData = baseData;
		subImg.strideInBytes = strideInBytes;
		subImg.ownsMemory = ownsMemory;
		subImg.width = w;
		subImg.height = h;

		// XXX: Breaks const-correctness...but not sure how else to structure this?
		subImg.imageStart = (void*)GetPixelPtrNoABC(x, y);

		subImg.Retain();

		return subImg;
	}

	void DeepCopyTo(BNImage<_T, _Channels> outCopy) const {
		ASSERT(outCopy.width == width);
		ASSERT(outCopy.height == height);
		BNS_FOR_J(height) {
			memcpy(outCopy.GetPixelPtrNoABC(0, j), GetPixelPtrNoABC(0, j), width * PixelBytes);
		}
	}

	BNImage<_T, _Channels> GetDeepCopy() const {
		BNImage<_T, _Channels> cpy(width, height);
		DeepCopyTo(cpy);
		return cpy;
	}

	BNImage(int _w, int _h) {
		width = _w;
		height = _h;

		// We store a reference count in the first 4 bytes
		// TODO: alignment
		int totalBytes = sizeof(int) + (width * height * PixelBytes);

		void* alloc = malloc(totalBytes);
		*(int*)alloc = 1;
		baseData = ((char*)alloc) + sizeof(int);
		imageStart = baseData;

		strideInBytes = width * PixelBytes;
		ownsMemory = true;
	}

	BNImage(int _w, int _h, void* buffer, int stride = -1) {
		if (stride < 0) {
			stride = _w * PixelBytes;
		}

		width = _w;
		height = _h;
		strideInBytes = stride;
		baseData = buffer;
		imageStart = baseData;
		ownsMemory = false;
	}

	BNImage(const BNImage<_T, _Channels>& orig) {
		width = orig.width;
		height = orig.height;
		strideInBytes = orig.strideInBytes;
		baseData = orig.baseData;
		imageStart = orig.imageStart;
		ownsMemory = orig.ownsMemory;

		Retain();
	}

	inline _T* GetPixelPtr(int x, int y) {
		return (_T*)((const BNImage<_T, _Channels>*)this)->GetPixelPtr(x, y);
	}

	inline _T* GetPixelPtrNoABC(int x, int y) {
		return (_T*)((const BNImage<_T, _Channels>*)this)->GetPixelPtrNoABC(x, y);
	}
	
	inline const _T* GetPixelPtr(int x, int y) const {
		ASSERT(x >= 0 && x < width);
		ASSERT(y >= 0 && y < height);
		return GetPixelPtrNoABC(x, y);
	}

	inline const _T* GetPixelPtrNoABC(int x, int y) const {
		char* rowStart = (char*)(imageStart)+(strideInBytes * y);
		return (_T*)(rowStart + PixelBytes * x);
	}

	void operator=(const BNImage<_T, _Channels>& orig) {
		bool deRetainRelase = (orig.baseData != baseData);

		if (deRetainRelase) {
			Release();
		}

		width = orig.width;
		height = orig.height;
		strideInBytes = orig.strideInBytes;
		baseData = orig.baseData;
		imageStart = orig.imageStart;
		ownsMemory = orig.ownsMemory;

		if (deRetainRelase) {
			Retain();
		}
	}
};


struct BN_RGB {
	union {
		struct {
			unsigned char r;
			unsigned char g;
			unsigned char b;
		};

		unsigned char rgb[3];
	};
};

static_assert(sizeof(BN_RGB) == 3, "Check padding on RN_RGB");

struct BN_RGBA {
	unsigned char r;
	unsigned char g;
	unsigned char b;
	unsigned char a;
};


BNImage<unsigned char> LoadGSImageFromFile(const char* filename);


#endif
