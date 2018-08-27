/*******************************************************************************
* This project is the final part of the Seminars course. This is an            *
* implementation of the PatchMatch Stereo algorithm for 3D local stereo        *
* reconstruction. The algorithm is described in the paper:                     *
* Michael Bleyer, Christoph Rhemann and Carsten Rother. PatchMatch Stereo -    *
* Stereo Matching with Slanted Support Windows. In Jesse Hoey, Stephen McKenna *
* and Emanuele Trucco, Proceedings of the British Machine Vision Conference,   *
* pages 14.1-14.11. BMVA Press, September 2011.                                *
* http://dx.doi.org/10.5244/C.25.14                                            *
*******************************************************************************/

#include <string>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>

namespace optionslib = boost::program_options;
using std::string;


int main(int argc, char *argv[]) {

	// Parsing the command line
	optionslib::options_description optDescription("SPMatch. "
			"Stereo matching with slanted support windows");

	// Defining the options
	optDescription.add_options()
			("help,h", "Help")
			("stereo-images", optionslib::value<std::vector<string>>()->required(),
			"The left and right input images");
	optionslib::positional_options_description posOptDescription;
	posOptDescription.add("stereo-images", 2);

	// Parsing
	optionslib::variables_map vMap;
	optionslib::store(optionslib::command_line_parser(argc, argv)
			.options(optDescription).positional(posOptDescription).run(),
			vMap);
	optionslib::notify(vMap);

	if (vMap.count("help")) {
		std::cout << optDescription << std::endl;
		return 1;
	}

	// Read the images path
	std::vector<string> imagesPath = vMap["stereo-images"]
			.as<std::vector<string>>();
	if (imagesPath.size() < 2) {		// enough?
		throw std::runtime_error("Need two images");
	}
	for (string s: imagesPath) {		// existing?
		std::ifstream f(s);
		if (!f.good()) {
			throw std::runtime_error("File not found: " + s);
		}
	}

	return 1;
}
