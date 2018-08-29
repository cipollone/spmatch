
#pragma once

#include <iostream>
#include <memory>
#include <vector>


using std::string;
using std::unique_ptr;


/***************************************************
* > class Grid                                     *
* A class for storing objects in a 2D static array *
***************************************************/
template<class T>
class Grid {
	
	private:

		size_t rows;
		size_t cols;

		std::vector<T> data;

	public:

		// constr
		Grid(size_t rows, size_t cols);

		// methods
		const T& get(size_t r, size_t c) const { return data[r*cols+c]; }
		const T& get(size_t i) const { return data[i]; }
		size_t size(void) const { return data.size(); }

		void set(size_t r, size_t c, const T& val) { data[r*cols+c] = val; }
		void set(size_t i, const T& val) { data[i] = val; }

		// operators
		template<class S>
		friend std::ostream& operator<<(std::ostream& o, const Grid<S>& g);
		
};
