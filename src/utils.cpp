
#include "utils.hpp"


// > class Grid

/*****************************
* > Grid()                   *
* Empty grid with known size *
*****************************/
template<class T>
Grid<T>::Grid(size_t rows, size_t cols):
		rows(rows), cols(cols), data(rows*cols) {}


// print
template<class T>
std::ostream& operator<<(std::ostream& o, const Grid<T>& g) {

	for (size_t r = 0; r < g.rows; ++r) {
		for (size_t c = 0; c < g.cols; ++c) {
			if (c > 0) {
				o << ", ";
			}
			o << g.get(r,c);
		}
		o << ";\n";
	}
	return o;
}
