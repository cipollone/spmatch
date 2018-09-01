
#include "image.hpp"


// > class Image

/*****************************************
* > Image()                              *
* Constructs and image from a file path. *
*****************************************/
Image::Image(const string& imgPath):
		imgPath(imgPath), img(imgPath.c_str()),
		width(img.width()), height(img.height()), channels(img.spectrum()) {
	
	if (channels != 1 && channels != 3) {
		throw std::runtime_error(
				"Wrong image format: " + std::to_string(channels) +
				" channels; only RGB and Grayscale supported");
	}
}


/*******************************************************
* > Image()                                            *
* Constructs a new image with the given size.          *
*                                                      *
* Args:                                                *
*   width (size_t), height (size_t), channels (size_t) *
*******************************************************/
Image::Image(size_t width, size_t height, size_t channels):
		imgPath("new_img.png"), img(width, height, 1, channels),
		width(width), height(height), channels(channels) {

	if (channels != 1 && channels != 3) {
		throw std::runtime_error(
				"Wrong image format: " + std::to_string(channels) +
				" channels; only RGB and Grayscale supported");
	}
}


/*************************************************
* > display()                                    *
* Show the image in a new window                 *
*                                                *
* Args:                                          *
*   windowName (string): the title of the window *
*************************************************/
void Image::display(string windowName) const {
	CImgDisplay disp(img, windowName.c_str());
	while (!disp.is_closed()) {
		disp.wait();
	}
}


/*****************************************************************************
* > toGrayscale()                                                            *
* Creates a new grayscale image from the current one using the ``luminance'' *
* formula.                                                                   *
*                                                                            *
* Returns:                                                                   *
*   (Image): a new image with the same width and height.                     *
*****************************************************************************/
Image Image::toGrayscale(void) const {

	// If already gray return a new image with the same properties
	if (channels == 1) {
		Image copyImg(*this);
		return copyImg;
	}

	// Create the new image (uninitialized)
	Image grayscale(width, height, 1);
	
	// Compute the gray pixels
	for (size_t w = 0; w < width; ++w) {
		for (size_t h = 0; h < height; ++h) {
			double red = img(w, h, 0, 0);
			double green = img(w, h, 0, 1);
			double blue = img(w, h, 0, 2);

			double gray = red * 0.3 + green * 0.59 + blue * 0.11;
			grayscale.img(w, h) = gray;
		}
	}

	return grayscale;
}


// print
std::ostream& operator<<(std::ostream& out, const Image& img) {
	return out << "Image: " << img.imgPath << ", size: (" << img.width <<
		"," << img.height << "," << img.channels << ")";
}


// > class StereoImage
/*************************************************************************
* > StereoImage()                                                        *
* Constructor. Initialize an image for stereo matching.  Loads the image *
* and initialize the array of support windows for each pixel.            *
*                                                                        *
* Args:                                                                  *
*   imgPath (string): the path of the image to load.                     *
*************************************************************************/
StereoImage::StereoImage(const string& imgPath):
		Image(imgPath),
		pixelPlanes(img.width(), img.height(),
			Grid<PlaneFunction>::Order::WIDTH_HEIGHT) {}
		// TODO: width, height order


// > class StereoImagePair

/**********************************
* > displayBoth()                 *
* Show both images in two windows *
**********************************/
void StereoImagePair::displayBoth(void) {
	leftImg.display("Left image");
	rightImg.display("Right image");
}


/********************************************************************
* > computeDisparity()                                              *
* Computes the disparity map of the two images using the PatchMatch *
* Stereo algorithm.                                                 *
*                                                                   *
* Returns:                                                          *
*   (Image): the disparity map                                      *
********************************************************************/
Image StereoImagePair::computeDisparity(void) {
	// TODO: remove the placeholder
	return Image("../tests/cones/all.png");
}


// print
std::ostream& operator<<(std::ostream& out, const StereoImagePair& pair) {
	return out << "StereoImagePair:\n  " << pair.leftImg << "\n  " <<
		pair.rightImg << std::endl;
}
