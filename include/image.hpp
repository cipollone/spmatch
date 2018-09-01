
#pragma once

#include <string>
#include <iostream>
#include <memory>

#include "geometry.hpp"
#include "utils.hpp"

#include <CImg.h>         // NOTE: including this before geometry.hpp
                          // (therefore Eigen) results in a compilation error
													// because of X11.h

using std::string;
using namespace cimg_library;


/************************************
* > class Image                     *
* A wrapper class for the CImg type *
************************************/
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
		void display(string windowName="") const;
		void write(void) { img.save(imgPath.c_str()); }
		Image toGrayscale(void) const;

		// methods
		void setPath(const string& path) { imgPath = path; }

		// operators
		Image& operator=(const Image&) = delete;
		friend std::ostream& operator<<(std::ostream& o, const Image& i);

};


class StereoImage: public Image {

	private:

		Grid<PlaneFunction> pixelPlanes;
		// TODO: methods for grid?

	public:

		// constr
		StereoImage(const string& imgPath);			// copy implicitly deleted

};


/***************************************
* > class StereoImagePair              *
* Represents a couple of StereoImages. *
* See .cpp file                        *
***************************************/
class StereoImagePair {
	
	private:

		StereoImage leftImg;
		StereoImage rightImg;

	public:

		// constr
		StereoImagePair(const string& leftImgPath, const string& rightImgPath):
				leftImg(leftImgPath), rightImg(rightImgPath) {}

		
		// methods
		void displayBoth(void);
		Image computeDisparity(void);

		// operators
		friend std::ostream& operator<<(std::ostream& o, const StereoImagePair& p);

};
