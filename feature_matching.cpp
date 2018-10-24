
#include "feature_matching.h"

void MatchFeaturesBasic(const Vector<BNFastKeyPoint>& srcKpts, const Vector<BNORBDescriptor>& srcDescriptors,
						const Vector<BNFastKeyPoint>& targetKpts, const Vector<BNORBDescriptor>& targetDescriptors,
						int matchThreshold, float nnTestRatio,
						Vector<BNFeatureMatch>* outMatches) {
	ASSERT(srcKpts.count == srcDescriptors.count);
	ASSERT(targetKpts.count == targetDescriptors.count);

	// TODO: Use nnTestRatio

	Vector<BNFeatureMatch> matches;

	const float boundingBoxRadiusSqr = BNS_SQR(50.0f);

	BNS_VEC_FOR_I(srcKpts) {
		int bestIdx = -1;
		int bestMatchDistance = INT_MAX;
		int secondBestMatchDistance = INT_MAX;

		BNLM::Vector2f imagePt1 = srcKpts.data[i].imagePoint;

		BNS_VEC_FOR_J(targetKpts) {
			BNLM::Vector2f imagePt2 = targetKpts.data[j].imagePoint;

			// TODO: THIS ALL NEEDS TO BE BETTER BLEEGFDG
			if ((imagePt1 - imagePt2).SquareMag() < boundingBoxRadiusSqr) {
				int dist = BNORBDistance(srcDescriptors.data[i], targetDescriptors.data[j]);
				if (dist < bestMatchDistance) {
					secondBestMatchDistance = bestMatchDistance;
					bestMatchDistance = dist;
					bestIdx = j;
				}
				else if (dist < secondBestMatchDistance) {
					secondBestMatchDistance = dist;
				}
			}
		}

		if (bestMatchDistance < matchThreshold && bestMatchDistance < (int)(secondBestMatchDistance * nnTestRatio)) {
			matches.PushBack(BNFeatureMatch(i, bestIdx));
		}
	}

	outMatches->Swap(matches);
}


