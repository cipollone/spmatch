# SPMatch, a PatchMatch Stereo implementation

This is a simple implementation of the PatchMatch algorithm for stereo reconstruction.
The algorithm is described in:
Michael Bleyer, Christoph Rhemann and Carsten Rother. *PatchMatch Stereo - Stereo Matching with Slanted Support Windows*. In Jesse Hoey, Stephen McKenna and Emanuele Trucco, Proceedings of the British Machine Vision Conference, pages 14.1-14.11. BMVA Press, September 2011.

This software closely follows the description of this paper. This is not an efficient implementation. There are some differences, however, in the Random Initialization and Plane Refinement steps. These changes allow to sample versors with uniform distribution in the required domains.

## Dependencies:
* libboost-program-options-dev (1.62)
* cimg-dev (1.7.9)
* imagemagick (8:6.9.7)
* libeigen3-dev (3.3.2)

The version numbers are the ones that have been used. Any future version with the same API is fine.

## Build:
First install all the dependencies. Then, you can build using CMake, as:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Usage:
Run:
```
spmatch <left-image> <right-image>
```
The left and right images must be RGB images. The outputs are two new images, "disparityL.<ext>" and "disparityR.<ext>", with the computed disparity map from the two views. These images are nomalized in [0,255], because they are just visual representations of the result.
The actual disparity will be written in two other files: "disparityL.csv" and "disparityR.csv". The format of each line is:
```
<p_w>, <p_h>, <disparity>
```
This means that the disparity value of pixel (p_w, p_h)  is given by the real number "disparity".

There are many options available. Run `spmatch --help` for alist of all available options. Some of them (e.g. -i and -w) affect excecution time and accuracy.
