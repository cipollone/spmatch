
#pragma once

#include <string>
#include <iostream>
#include <memory>
#include <CImg.h>

using std::string;
using std::unique_ptr;
using namespace cimg_library;


class Image {

	private:

		string imgPath;

		unique_ptr<CImg<double>> img;   // NOTE: using double type for pixels

		int width;
		int height;
		int channels;

	public:

		Image(const string& imgPath);
		Image(const Image&) = delete;     // no copies
		Image(Image&& i) = default;

		void display(string windowName="");

		Image& operator=(const Image&) = delete;
		friend std::ostream& operator<<(std::ostream& o, const Image& i);

};


class StereoImage: public Image {

	public:

		StereoImage(const string& imgPath): Image(imgPath) {}

};


class StereoImagePair {
	
	private:

		StereoImage leftImg;
		StereoImage rightImg;

	public:

		StereoImagePair(const string& leftImgPath, const string& rightImgPath):
				leftImg(leftImgPath), rightImg(rightImgPath) {}

		void displayBoth(void);

		friend std::ostream& operator<<(std::ostream& o, const StereoImagePair& p);

};
