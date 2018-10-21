#ifndef FEATURE_MATCHING_H_
#define FEATURE_MATCHING_H_

struct BNFeatureMatch {
	int srcIdx;
	int targetIdx;

	BNFeatureMatch(int _srcIdx, int _targetIdx) {
		srcIdx = _srcIdx;
		targetIdx = _targetIdx;
	}
};


#endif
