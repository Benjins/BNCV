#include "external/BNLM/core.h"





BNLM::Matrix3f ComputeInitialIntrinsicsMatrixFromHomographies(const Vector<BNLM::Matrix3f>& homographies) {

	BNLM::MatrixXf BSystem(2 * homographies.count, 6);
	BSystem.ZeroOut();

	BNS_VEC_FOR_I(homographies) {
		int row1 = 2 * i;
		int row2 = 2 * i + 1;

		auto H = homographies.data[i];

		// Setup the constraints for each row:
		// We have two constraints per homography (unit-length constraint and orthoganl constraint)
		#define SETUP_VEC(vec, i, j) do {                   \
			vec(0) = H(0, i) * H(0, j);                     \
			vec(1) = H(0, i) * H(1, j) + H(1, i) * H(0, j); \
			vec(2) = H(1, i) * H(1, j);                     \
			vec(3) = H(2, i) * H(0, j) + H(0, i) * H(2, j); \
			vec(4) = H(2, i) * H(1, j) + H(1, i) * H(2, j); \
			vec(5) = H(2, i) * H(2, j);                     \
		} while(0)

		{
			BNLM::Vector<float, 6> scratchVec1;
			SETUP_VEC(scratchVec1, 0, 1);
			BNS_FOR_I(6) { BSystem(row1, i) = scratchVec1(i); }
		}

		{
			BNLM::Vector<float, 6> scratchVec1, scratchVec2;
			SETUP_VEC(scratchVec1, 0, 0);
			SETUP_VEC(scratchVec2, 1, 1);
			BNS_FOR_I(6) { BSystem(row2, i) = scratchVec1(i) - scratchVec2(i); }
		}
	}

	BNLM::MatrixXf U, V;
	BNLM::VectorXf sigma;
	BNLM::SingularValueDecomposition(BSystem, &U, &sigma, &V);

	ASSERT(V.rows == 6);
	ASSERT(V.cols == 6);

	BNLM::Vector<float, 6> BVec;
	BNS_FOR_I(6) {
		BVec(i) = V(i, 5);
	}

	BNLM::Matrix3f B;
	B(0, 0) = BVec(0);
	B(0, 1) = BVec(1);
	B(1, 0) = BVec(1);
	B(1, 1) = BVec(2);
	B(0, 2) = BVec(3);
	B(2, 0) = BVec(3);
	B(1, 2) = BVec(4);
	B(2, 1) = BVec(4);
	B(2, 2) = BVec(5);

	const float scaleFactor = 512.0f;

	float cy = (-B(0, 0) * B(1, 2)) / (B(0, 0) * B(1, 1));
	printf("cy: %f\n", cy * scaleFactor);

	float lambda = B(2, 2) - (BNS_SQR(B(0, 2)) + cy * (B(0, 1) * B(0, 2) - B(0, 0) * B(1, 2))) / B(0, 0);
	float fx = sqrt(lambda / B(0, 0));
	printf("fx: %f\n", fx * scaleFactor);

	float fy = sqrt(lambda * B(0, 0) / (B(0, 0) * B(1, 1) - BNS_SQR(B(0, 1))));
	printf("fy: %f\n", fy * scaleFactor);

	float cx = -B(0, 2) * fx*fx / lambda;
	printf("cx: %f\n", cx * scaleFactor);

	return BNLM::Matrix3f();
}



