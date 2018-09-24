
#include "numbers.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>
#include <algorithm>

using std::vector;
using std::pair;
using std::string;


/************************************************************************
* > weightedMedian()                                                    *
* Return the weighted median value.                                     *
* NOTE: cost is O(n log(n)), where n is the number of values.           *
*   The input is copied in memory.                                      *
*                                                                       *
* Args:                                                                 *
*   values (vector<double>): the vector of values                       *
*   weighs (vector<double>): the vector of weighs (their sum must be 1) *
*                                                                       *
* Returns:                                                              *
*   (double): the weighted median. (averaged, if with two candidates)   *
************************************************************************/
double weightedMedian(const vector<double>& values,
		const vector<double>& weights) {

	// Using epsilon for weights
	auto areEqualWeights = [] (const double& w1, const double& w2) -> bool {
		return std::abs(w1 - w2) <= std::numeric_limits<float>::epsilon();
	};

	// checks
	if (values.size() == 0) {
		throw std::invalid_argument("weightedMedian(). emtpy input");
	}
	if (values.size() != weights.size()) {
		throw std::invalid_argument("weightedMedian(). inconsistent sizes");
	}
	double wSum = 0;
	for (auto w: weights) {
		if (w <= 0) {
			throw std::domain_error("weightedMedian(). Weights must be positive.");
		}
		wSum += w;
	}
	if (!areEqualWeights(wSum, 1)) {
		throw std::invalid_argument(string() +
				"weightedMedian(). the sum of all weights must be 1. It's " +
				std::to_string(wSum));
	}

	// Copy the input into a single vector
	vector<pair<double, double>> vec;
	for (size_t i = 0; i < values.size(); ++i) {
		vec.push_back(std::make_pair(values[i], weights[i]));
	}

	// Sort by values
	std::sort(vec.begin(), vec.end(),
			[] (pair<double,double> p1, pair<double,double> p2)
			{ return p1.first < p2.first; }
	);

	// Start from the left
	double wSumL = 0, wSumR = 1;

	// Scan the vector until we find 0.5 of the accumulated weight
	for (size_t i = 0; i < vec.size(); ++i) {
		auto& val = vec[i];

		// Base case: two median values.
		if (areEqualWeights(wSumL, wSumR)) {  // vec[i] is the upper median
			return (vec[i-1].first + vec[i].first)/2;
		}

		// Moving to the right wSumR
		wSumR -= val.second;

		// Base case: single median value
		if (wSumL < 0.5 && wSumR < 0.5 &&					// is a median &&
				!areEqualWeights(wSumR, 0.5)) {       // not a lower median
			return val.first;
		}
		
		// Moving to the right wSumL
		wSumL += val.second;
	}

	// never happens
	throw std::logic_error("weightedMedian(). no number selected as output");
}
