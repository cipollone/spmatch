
#pragma once

#include <random>

#include "params.hpp"


/***********************************************************
* > class RandomDevice                                     *
* A single-instance class that serves as the global random *
* numbers generator.                                       *
***********************************************************/
class RandomDevice {

	private:

		std::random_device device;

	public:

		std::default_random_engine engine;

	private:

		// Can't instantiate directly
		RandomDevice(void) {
			if (!params.USE_PSEUDORAND) {
				engine.seed(device());
			}
		}

	public:

		RandomDevice(const RandomDevice&) = delete;
		void operator=(const RandomDevice&) = delete;

		// Getter of the global object
		static RandomDevice& getGenerator(void) {
			static RandomDevice generator;
			return generator;
		}
};


// see .cpp
double weightedMedian(const std::vector<double>& values,
		const std::vector<double>& weights); 
