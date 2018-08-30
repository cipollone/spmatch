
#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <random>


using std::string;
using std::unique_ptr;


/***************************************************
* > class Grid                                     *
* A class for storing objects in a 2D static array *
***************************************************/
template<typename T>
class Grid {
	
	private:

		size_t nRows;
		size_t nCols;

		std::vector<T> data;

	public:

		// constr
		Grid(size_t rows, size_t cols):
				nRows(rows), nCols(cols), data(rows*cols) {}
		Grid(size_t rows, size_t cols, const T& val):
				nRows(rows), nCols(cols), data(rows*cols, val) {}

		// methods
		const T& get(size_t r, size_t c) const { return data[r*nCols+c]; }
		const T& get(size_t i) const { return data[i]; }
		size_t size(void) const { return data.size(); }
		size_t rows(void) const { return nRows; }
		size_t cols(void) const { return nCols; }

		void set(size_t r, size_t c, const T& val) { data[r*nCols+c] = val; }
		void set(size_t i, const T& val) { data[i] = val; }

		// operators
		template<typename S>
		friend std::ostream& operator<<(std::ostream& o, const Grid<S>& g);
		
};


// > class Grid

// print
template<typename T>
std::ostream& operator<<(std::ostream& o, const Grid<T>& g) {

	for (size_t r = 0; r < g.nRows; ++r) {
		for (size_t c = 0; c < g.nCols; ++c) {
			if (c > 0) {
				o << ", ";
			}
			o << g.get(r,c);
		}
		o << ";\n";
	}
	return o;
}


/**************************
* > class RandomDevice    *
* A global random device  *
**************************/
class RandomDevice {

	public:
		std::random_device rndDev;
};

