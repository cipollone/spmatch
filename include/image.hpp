
#pragma once

#include <string>

#include "geometry.hpp"
#include "utils.hpp"

#include <CImg.h>         // NOTE: including this before geometry.hpp
                          // (therefore Eigen) results in a compilation error
													// because of X11.h

using std::string;
using namespace cimg_library;


/********************************************************
* > class Image                                         *
* A wrapper class for the CImg type.                    *
* Pixels are stored as double in [0, 255] in RGB space. *
* See the comments in .cpp file.                        *
********************************************************/
class Image {

	protected:

		string imgPath;

		CImg<double> img;            // NOTE: using double type for pixels

		size_t width;
		size_t height;
		size_t channels;

	public:

		// constr
		explicit Image(const string& imgPath);
		explicit Image(size_t width, size_t height, size_t channels);
		explicit Image(const Image&) = default;
		Image(Image&&) = default;         // move: allows to return images

		// const methods
		double at(double w, double h, size_t c) const;
		const CImg<double> getCImg(void) const { return img; }
		void display(string windowName="") const;
		void write(void) const { img.save(imgPath.c_str()); }
		Image toGrayscale(void) const;

		// methods
		void setPath(const string& path) { imgPath = path; }

		// operators
		Image& operator=(const Image&) = delete;
		double operator()(size_t w, size_t h, size_t c) const { return img(w,h,0,c); }
		double operator()(size_t w, size_t h) const { return img(w,h); }
		friend std::ostream& operator<<(std::ostream& o, const Image& i);

};
