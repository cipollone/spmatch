
#include "image.hpp"


// > class Image

/*****************************************
* > Image()                              *
* Constructs and image from a file path. *
*****************************************/
Image::Image(const string& imgPath):
		imgPath(imgPath), img(new CImg<double>(imgPath.c_str())),
		width(img->width()), height(img->height()), channels(img->spectrum()) {
	
	if (channels != 1 || channels != 3 || channels != 4) {
		std::runtime_error("Wrong image format. Grayscale, RGB, RGBA supported");
	}
}


/*************************************************
* > display()                                    *
* Show the image in a new window                 *
*                                                *
* Args:                                          *
*   windowName (string): the title of the window *
*************************************************/
void Image::display(string windowName) {
	CImgDisplay disp;
	disp.display(*img);
	while (!disp.is_closed()) {
		disp.wait();
	}
}


// print
std::ostream& operator<<(std::ostream& out, const Image& img) {
	return out << "Image: " << img.imgPath << ", size: (" << img.width <<
		"," << img.height << "," << img.channels << ")";
}



// > class StereoImagePair

/**********************************
* > displayBoth()                 *
* Show both images in two windows *
**********************************/
void StereoImagePair::displayBoth(void) {
	leftImg.display("Left image");
	rightImg.display("Right image");
}

// print
std::ostream& operator<<(std::ostream& out, const StereoImagePair& pair) {
	return out << "StereoImagePair:\n  " << pair.leftImg << "\n  " <<
		pair.rightImg << std::endl;
}
