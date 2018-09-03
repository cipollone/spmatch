
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
		throw std::logic_error(
				"Wrong image format: " + std::to_string(channels) +
				" channels; only RGB and Grayscale supported");
	}
}


/**************************************************************************
* > at()                                                                  *
* Returns the intensity of the c channel. The result is obtained with     *
* bilinear interpolation in the x,y directions. NOTE: bounds are checked. *
* For fast, discrete lookup, use the () operator.                         *
**************************************************************************/
double Image::at(double w, double h, size_t c) const {

	// checks
	if (c >= channels) {
		throw std::logic_error("Wrong channel, " + std::to_string(c) +
				" of [0, " + std::to_string(channels-1) + "]");
	}
	if (w >= width || h >= height || w < 0 || h < 0) {
		throw std::logic_error("Wrong coorinate, (" + std::to_string(w) +
				", " + std::to_string(h) + ") of (0, 0) -- (" +
				std::to_string(width-1) + ", " + std::to_string(height-1) + ")");
	}

	// Get extremes
	size_t wLow = std::floor(w);
	size_t hLow = std::floor(h);
	size_t wHigh = std::ceil(w);
	size_t hHigh = std::ceil(h);
	double x = w - wLow;
	double y = h - hLow;

	double z00 = img(wLow, hLow, c);
	double z01 = img(wLow, hHigh, c);
	double z10 = img(wHigh, hLow, c);
	double z11 = img(wHigh, hHigh, c);

	// linear
	double z = z00 * (1 - x) * (1 - y) + 
			z10 * x * (1 - y) + 
			z01 * (1 - x) * y + 
			z11 * x * y;
	return z;
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
