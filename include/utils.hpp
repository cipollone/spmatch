
#pragma once

#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <random>


using std::string;
using std::unique_ptr;


/**************************************************************************
* > class Grid                                                            *
* A template class for storing objects in a 2D static array. Every method *
* that accept the two indexes of an element can follow one of two         *
* conventions: (row, column) or (width, height) depending on the Order    *
* constructor parameter.  Elements are indexed from 0.                    *
**************************************************************************/
template<typename T>
class Grid {

	public:

		enum class Order { ROWS_COLS, WIDTH_HEIGHT };
	
	private:

		// members
		const Order order;
		size_t nRows;
		size_t nCols;

		std::vector<T> data;

	private:

		// private methods
		bool reorderArgs(size_t& i1, size_t& i2) const;

	public:

		// constr
		Grid(size_t rows, size_t cols, Order order);
		Grid(size_t rows, size_t cols, Order order, const T& val);

		// const methods
		size_t size(void) const { return data.size(); }
		size_t rows(void) const { return nRows; }
		size_t cols(void) const { return nCols; }
		size_t width(void) const { return nCols; }
		size_t height(void) const { return nRows; }
		const T& get(size_t i1, size_t i2) const;

		// operators
		T& operator()(size_t r, size_t c);
		T& operator()(size_t i) { return data[i]; }
		template<typename S>
		friend std::ostream& operator<<(std::ostream& o, const Grid<S>& g);
};


// > class Grid

/***************************************************
* > reorderArgs()                                  *
* Swaps the two arguments if order == WIDTH_HEIGHT *
*                                                  *
* Args:                                            *
*   i1 (size_t&): reference to the first arg       *
*   i2 (size_t&): reference to the second arg      *
*                                                  *
* Return:                                          *
*   (bool): true if swapped                        *
***************************************************/
template<typename T>
bool Grid<T>::reorderArgs(size_t& i1, size_t& i2) const {
	if (this->order == Order::WIDTH_HEIGHT) {
		std::swap(i1, i2);
		return true;
	}
	return false;
}

/****************************************************************************
* > Grid()                                                                  *
* Constructor. A grid of elements with their default value of size (n1, n2) *
*                                                                           *
* Args:                                                                     *
*   n1 (size_t): lenght of the first coordinate                             *
*   n2 (size_t): lenght of the second coordinate                            *
*   order (Order): the interpretation of each couple of arguments           *
****************************************************************************/
template<typename T>
Grid<T>::Grid(size_t n1, size_t n2, Order order):
		order(order) {
		
	reorderArgs(n1, n2);
	nRows = n1;
	nCols = n2;
	data.assign(nRows*nCols, T());
}


/******************************************************************
* > Grid()                                                        *
* Constructor. A grid of (n1, n2) elements with given value.      *
*                                                                 *
* Args:                                                           *
*   n1 (size_t): lenght of the first coordinate                   *
*   n2 (size_t): lenght of the second coordinate                  *
*   order (Order): the interpretation of each couple of arguments *
*   val (const T&): the value of each entry                       *
******************************************************************/
template<typename T>
Grid<T>::Grid(size_t n1, size_t n2, Order order, const T& val):
		order(order) {
		
	reorderArgs(n1, n2);
	nRows = n1;
	nCols = n2;
	data.assign(nRows*nCols, val);
}


/*********************************************************
* > get()                                                *
* Args:                                                  *
*   i1 (size_t): first index                             *
*   i2 (size_t): second index                            *
*                                                        *
* Returns:                                               *
*   (const T&): reference to element at position (i1,i2) *
*********************************************************/
template<typename T>
const T& Grid<T>::get(size_t i1, size_t i2) const {
	reorderArgs(i1, i2);
	return data[i1*nCols+i2];
}


/***************************************************
* > operator()()                                   *
* Args:                                            *
*   i1 (size_t): first index                       *
*   i2 (size_t): second index                      *
*                                                  *
* Returns:                                         *
*   (T&): reference to element at position (i1,i2) *
***************************************************/
template<typename T>
T& Grid<T>::operator()(size_t i1, size_t i2) {
	reorderArgs(i1, i2);
	return data[i1*nCols+i2];
}


// print
template<typename T>
std::ostream& operator<<(std::ostream& o, const Grid<T>& g) {

	size_t nR = g.nRows;
	size_t nC = g.nCols;
	g.reorderArgs(nR, nC);

	for (size_t r = 0; r < nR; ++r) {
		for (size_t c = 0; c < nC; ++c) {
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
