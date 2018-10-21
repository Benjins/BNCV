#ifndef FEATURES_H_
#define FEATURES_H_

#pragma once

#include "external/BNLM/core.h"

#include "BNImage.h"

// TODO: Should we SOA this instead?
struct BNFastKeyPoint {
	BNLM::Vector2f imagePoint;
	float rotation = 0.0f;
	float score = 0.0f;
};

struct BNORBDescriptor {
	union {
		unsigned char asUchar[32];
		unsigned int asUint[8];
	};
};

static_assert(sizeof(BNORBDescriptor) == 32, "Check ORB padding?");

// XOR, then count bits to get hamming distance word-by-word then sum
// Taken from http ://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
// TODO: Is const ref or value faster here?
// TODO: SIMD? Is it worth it? Should we have versions that take large arrays instead?
inline int BNORBDistance(const BNORBDescriptor& a, const BNORBDescriptor& b) {
	int dist = 0;
	BNS_FOR_I(8) {
		unsigned int v = a.asUint[i] ^ b.asUint[i];
		v = v - ((v >> 1) & 0x55555555);                    // reuse input as temporary
		v = (v & 0x33333333) + ((v >> 2) & 0x33333333);     // temp
		dist += ((v + (v >> 4) & 0xF0F0F0F) * 0x1010101) >> 24; // count
	}

	return dist;
}

void FindORBFeaturePointsInGSImage(BNImage<unsigned char, 1> img,
								   int threshold,
								   Vector<BNFastKeyPoint>* outKeypoints,
								   Vector<BNORBDescriptor>* outDescriptors);

#endif
