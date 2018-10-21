#ifndef FEATURE_MATCHING_H_
#define FEATURE_MATCHING_H_

#include "external/BNLM/CppUtils/vector.h"

#include "features.h"

struct BNFeatureMatch {
	int srcIdx;
	int targetIdx;

	BNFeatureMatch(int _srcIdx, int _targetIdx) {
		srcIdx = _srcIdx;
		targetIdx = _targetIdx;
	}
};


void MatchFeaturesBasic(const Vector<BNFastKeyPoint>& srcKpts, const Vector<BNORBDescriptor>& srcDescriptors,
						const Vector<BNFastKeyPoint>& targetKpts, const Vector<BNORBDescriptor>& targetDescriptors,
						int matchThreshold, float nnTestRatio,
						Vector<BNFeatureMatch>* outMatches);



#endif
