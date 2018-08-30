# SPMatch, a PatchMatch Stereo implementation

This is a basic implementation the PatchMatch algorithm for stereo reconstruction.

The algorithm is described in:
Michael Bleyer, Christoph Rhemann and Carsten Rother. *PatchMatch Stereo -
Stereo Matching with Slanted Support Windows*. In Jesse Hoey, Stephen McKenna
and Emanuele Trucco, Proceedings of the British Machine Vision Conference,
pages 14.1-14.11. BMVA Press, September 2011.

## Dependencies:
* libboost
* CImg
* imagemagic
* libeigen3

## Usage:
Run:

	spmatch <left-image>.<ext> <right-image>.<ext>

The result is a new image, <left-image>\_disp.<ext>, with the computed
disparity.
