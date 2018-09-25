# SPMatch, a PatchMatch Stereo implementation

This is a basic implementation the PatchMatch algorithm for stereo reconstruction.

The algorithm is described in:
Michael Bleyer, Christoph Rhemann and Carsten Rother. *PatchMatch Stereo -
Stereo Matching with Slanted Support Windows*. In Jesse Hoey, Stephen McKenna
and Emanuele Trucco, Proceedings of the British Machine Vision Conference,
pages 14.1-14.11. BMVA Press, September 2011.

## Dependencies:
* libboost (1.62)
* cimg-dev (1.7.9)
* imagemagick (8:6.9.7)
* libeigen3-dev (3.3.2)

The version numbers are the ones that have been used. Any future version with
the same API is fine.

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
The left and right images must be RGB images.  The result is a new image,
"disparity.png", with the computed disparity. Run `spmatch --help` for a
list of all available options.
