
#pragma once

#include <string>
#include <iostream>
#include <memory>
#include <CImg.h>

using std::string;
using namespace cimg_library;


class Image {

	private:

		string imgPath;

		CImg<double> img;            // NOTE: using double type for pixels

		int width;
		int height;
		int channels;

	public:

		// constr
		Image(const string& imgPath);
		Image(const Image&) = delete;     // no copies
		Image(Image&&) = default;         // move: allows to return images

		// functions
		void display(string windowName="");

		// operators
		Image& operator=(const Image&) = delete;
		friend std::ostream& operator<<(std::ostream& o, const Image& i);

};


class StereoImage: public Image {

	private:

		// TODO: a grid of planes



	public:

		// constr
		StereoImage(const string& imgPath): Image(imgPath) {}

};


class StereoImagePair {
	
	private:

		StereoImage leftImg;
		StereoImage rightImg;

	public:

		// constr
		StereoImagePair(const string& leftImgPath, const string& rightImgPath):
				leftImg(leftImgPath), rightImg(rightImgPath) {}

		
		// functions
		void displayBoth(void);
		Image computeDisparity(void);

		// ops
		friend std::ostream& operator<<(std::ostream& o, const StereoImagePair& p);

};
