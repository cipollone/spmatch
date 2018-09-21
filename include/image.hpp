
#pragma once

#include <string>

#include "geometry.hpp"
#include "utils.hpp"

#include <CImg.h>         // NOTE: including this before geometry.hpp
                          // (therefore Eigen) results in a compilation error
													// because of X11.h

using std::string;
using namespace cimg_library;


/*******************************************************************
* > class Image                                                    *
* A wrapper class for the CImg type.                               *
* Pixels are stored as double in [0, 255] in RGB space.            *
* This class uses move semantics: it is possible to return images. *
* See the comments in .cpp file.                                   *
*******************************************************************/
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
		explicit Image(size_t width, size_t height, size_t channels, double val);
		explicit Image(const Image&) = default;
		Image(Image&&) = default;
		Image(CImg<double>&&);

		// const methods
		size_t size(unsigned dim) const;
		double get(size_t w, size_t h, size_t c) const { return img(w,h,0,c); }
		double get(size_t w, size_t h) const { return img(w,h); }
		double at(double w, double h, size_t c) const;
		const CImg<double>& getCImg(void) const { return img; }
		void display(void) const { img.display(); }
		void write(void) const { img.save(imgPath.c_str()); }
		Image toGrayscale(void) const;

		// methods
		Image& setPath(const string& path) { imgPath = path; return *this; }
		Image& normalize(void) { img.normalize(0, 255); return *this; }

		// operators
		Image& operator=(const Image&) = delete;
		Image& operator=(CImg<double>&&);
		double& operator()(size_t w, size_t h, size_t c) { return img(w,h,0,c); }
		double& operator()(size_t w, size_t h) { return img(w,h); }
		friend std::ostream& operator<<(std::ostream& o, const Image& i);

};
