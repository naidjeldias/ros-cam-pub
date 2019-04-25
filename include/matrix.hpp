
/**
 * mat/matrix.hpp
 *
 *  Created on: 18 Oct 2017
 *      Author: Gustavo T. L., gustavo
 */

/**
 * @brief 
 * 
 * @file matrix.hpp
 * @author gustavoengdm "Gustavo Teodoro Laureano"
 * @date 2018-07-16
 */
#ifndef _MCORE_MATRIX_HPP_
#define _MCORE_MATRIX_HPP_

#include <iostream> 
#include <iomanip>

#include "msg.h"
#include "mem.hpp"

#include "func_obj.hpp"
#include "baop.hpp"

namespace mcore {

	template<typename T> struct matrix {
		private:
			/**
			 * Pointer of first element at allocated memory.
			 * When mcore::matrix is a view, this pointer is NULL.
			 */
			T * const array_ptr = 0x0;

		public:
			T * const data = 0x0; /**<	Pointer for the first usable element.
			 This pointer is the start of usable array, that is,
			 it is not necessarily the first address of the allocated memory. */
			
			const int nrows = 0; /**< number of rows */
			const int ncols = 0; /**< number of columns */
			const int depth = 0; /**< Depth of matrix. It can be related with third dimension of a image.
			 When depth is 1 we have a planar image. */
			
			int const stride = 0; /**< Stride is the number of elements between two adjacent rows. */
			int const size = 0; /**< number of elements of matrix. size = nrows*ncols*depth */

			matrix() {}

			/**
			 * Default Constructor
			 * ---------------------
			 *
			 * This constructor creates a new _*nr*_ by _*nc*_
			 * mcore::matrix \f$ \left\{ \mathbf{A}_{nr\times nc} \right\} \f$
			 *
			 * @param nr : number of rows
			 * @param nc : number of columns
			 */
			matrix(int nr, int nc, int dp = 1) {
				T * ptr;
				if (nr < 1 || nc < 1) return;

				ptr = new T[nr * nc * dp];
				if (ptr != 0x0) __set_parameters(ptr, ptr, nr, nc, dp, nc * dp);
			}

			/**
			 * Constructor for a data viewer
			 * --------------
			 *
			 * This constructor make a view of a input matrix.
			 *
			 * With this construction the object matrix will not be destroyed.
			 *
			 * @param m : input matrix
			 * @param r : row of the first element of the view
			 * @param c : column of the first element of the view
			 * @param nr : number of rows of the view
			 * @param nc : number of columns of the view
			 */
			matrix( matrix & m, int r, int c, int nr, int nc) {
				
				__assert_error( m.data == NULL, "Input matrix does not have data!\n" );
				__assert_error( r < 0 || c < 0 || nr > (m.nrows - r) || nc > (m.ncols - c), "Invalid dimensions!\n" );

				__set_parameters_as_viewer( &m.at(r,c), nr, nc, m.depth, m.stride);
			}

			static matrix<T> * create( void ) {
				matrix<T> * M = new matrix<T>();
				return M;
			}

			static matrix<T> * create( int rows, int cols ) {
				matrix<T> * M = new matrix<T>(rows, cols);
				return M;
			}


			/**
			 * @brief Verify if this object is a view
			 * 
			 * @return true if this object is a view
			 * @return false if this object is not a view
			 */
			bool is_a_viewer(void) const {
				return (array_ptr == 0x0 && data != 0x0);
			}

			/**
			 * Destructor.
			 *
			 * Destroy allocated data if object is not a viewer.
			 */
			~matrix() {
				if ( is_a_viewer() == false ) {
					//delete[] array_ptr;
					this->clear();
				}
				__set_parameters(0x0, 0x0, 0, 0, 0, 0);
			}

			/**
			 * @brief Computes the linear array index
			 * 
			 * @param r row number
			 * @param c column number
			 * @param dp depth number
			 * @return int the linear index
			 */
			int idx(int r, int c, int dp = 0) const {
				return (r*stride + c*depth) + dp;
			}

			/**
			 * @brief Gets the element (r,c,dp)
			 * 
			 * @param r row number
			 * @param c column number
			 * @param dp depth number
			 * @return T& the element at position (r,c,dp)
			 */
			T & at(int r, int c, int dp = 0) {
				return data[ (r*stride + c*depth) + dp ];
			}

			T & at( int i ) {
				if( !is_a_viewer() ) return data[i];

				int st, r, c, dp;

				st = ncols*depth;
				r = i / st;
				c = i % st / depth;
				dp = i % st % depth;

				return data[ (r*stride + c*depth) + dp ];
			}

			T & operator()( int i) {
				return *(data + (i/ncols*stride + i%ncols) );
			}

			T & operator()( int r, int c, int dp = 0 ) {
				return data[ (r*stride + c*depth) + dp ];
			}
	
			/**
			 * @brief Get the number of elements in the array
			 * 
			 * @return int the number of elements in the array.
			 */
			int length(void) const {
				return nrows * ncols * depth;
			}

			bool equal(const mcore::matrix<T> & m) const {
				return (this == &m || this->array_ptr == m.array_ptr || this->data == m.data);
			}

			template<typename T2>
			bool equal(const mcore::matrix<T2> & m) const {
				return false;
			}

			void flip(void) {
				int i, s = length();
				T v;
				for (i = 0; i <= s / 2; i++) {
					v = data[i];
					data[i] = data[s - 1 - i];
				}
			}

			void normalize(T min, T max) {
				double M, m;
				int i, j, k;
				M = m = data[0];
				for (i = 0; i < nrows; i++) {
					for (j = 0; j < ncols; j++) {
						k = i * ncols + j;
						if (data[k] > M) M = data[k];
						if (data[k] < m) m = data[k];
					}
				}
				for (i = 0; i < nrows; i++) {
					for (j = 0; j < ncols; j++) {
						k = i * ncols + j;
						data[k] = (data[k] - m) / (M - m);
						data[k] = min + data[k] * (max - min);
					}
				}				
			}

			matrix<T> & operator=( double v ) {

				if( is_a_viewer() ) {

					if( this->depth == 1 ) {
						mcore::impl::arrayset2d( this->data, this->nrows, this->ncols, this->depth, this->stride, v );
					} else {
						mcore::impl::arrayset3d( this->data, this->nrows, this->ncols, this->depth, this->stride, v );
					}

				} else {
					this->set_size(1,1,1);
					*this->data = v;
				}

				return *this;
			}

			template<typename Tin>
			matrix<T> & operator=(const mcore::matrix<Tin> & M) {

				if ( is_a_viewer() ) {
					__assert_error( M.length() != this->length() , "Dimensions are not compatible!\n");
				} else {
					this->set_size(M.nrows, M.ncols, M.depth);
				}

				if( M.depth == 1 ) {
					mcore::impl::arraycpy2d( this->data, this->stride, M.data, M.nrows, M.ncols, M.stride );
				} else {
					mcore::impl::arraycpy3d( this->data, this->stride, M.data, M.nrows, M.ncols, M.depth, M.stride );
				}

				return *this;
			}

			template<typename Tf>
			mcore::matrix<T> & operator=( const mcore::FuncObj<Tf> & mf ) {
				mf.get_derived().apply(*this);
				return *this;
			}

			// template<typename Tin>
			// matrix<T> & operator=(const mcore::matrix<Tin> & M) {
			// 	int i;
			// 	int s = M.length();
			// 	for (i = 0; i < s; i++) {
			// 		data[i] = (T) M.data[i];
			// 	}
			// 	return *this;
			// }

			/**
			 * @brief Print matrix elements into terminal
			 * 
			 * @param text A text to apear before the elements
			 * @param show_dim show matrix dimensions
			 * @param newline print a new line after elements
			 */
			void print(const char * text = 0x0, bool show_dim = true, bool newline = true) {

				if (text) std::cout << text;
				if (show_dim) std::cout << "(" << this->nrows << "," << this->ncols << "," << this->depth << ")" << std::endl;
				for( int d = 0; d < this->depth; d ++ ) {
					for (int r = 0; r < this->nrows; r++) {
						std::cout << "|";
						for (int c = 0; c < this->ncols; c++) {
							std::cout << std::setw(5) << this->at(r,c,d);
						}
						std::cout << "|" << std::endl;
					}
					for( int i = 0; i < ncols; i++ ) std::cout << "+----";
					std::cout << std::endl;
				}
				if (newline) std::cout << std::endl;
			}

			/**
			 * @brief Set a new size for the matrix object.
			 * 
			 * @param nr a valid number of rows.
			 * @param nc a valid number of columns.
			 * @param dp a valid number of depth. Default is 1.
			 * @return true if memory allocation is OK or false otherwise.
			 */
			void set_size(int nr, int nc, int dp = 1) {

				T * ptr = 0x0;

				if ( !is_a_viewer() ) {
					if (nrows * ncols * depth != nr * nc * dp) clear();
					else return;
				}
				ptr = new T[nr * nc * dp];
				__assert_error( ptr == NULL, "Can't allocate memory!\n");
				__set_parameters(ptr, ptr, nr, nc, dp, nc * dp);
			}

			void set_data(T * const data, int nr, int nc) {
				set_data( data, nr, nc, 1, nc);
			}

			void set_data(T * const data, int nr, int nc, int dp) {
				set_data( data, nr, nc, dp, nc);
			}

			void set_data(T * const data, int nr, int nc, int ndp, int st) {

				// if (data == NULL || nr < 1 || nc < 1) return false;
				// if (ndp < 1) return false;
				// if (st < 1) return false;

				__set_parameters_as_viewer(data, nr, nc, ndp, st);
			}

			void reshape(int nr, int nc) {

				__assert_error( nr*nc != nrows*ncols, "\nCan't reshape matrix(%d,%d) to matrix(%d,%d)\n\n", nrows, ncols, nr, nc);

				__assert_error( is_a_viewer(), "The matrix is a viewer and it can not be reshaped!\n" );

				if (nrows == 1 || ncols == 1) {
					util::mem::rw(nrows) = nc;
					util::mem::rw(ncols) = nr;
				} else {
					// TODO:
					__assert_error( true, "\nReshape is not available yet.\n\n");
				}
			}

			matrix<T> row( int r ) {
				return __create_viewer( r, 0, 1, this->ncols );
			}
			matrix<T> col( int c ) {
				return __create_viewer( 0, c, this->nrows, 1 );
			}
			matrix<T> submat( int r, int c, int nr, int nc ) {

				__assert_error( r < 0 || c < 0, "Invalid indexing!\n" );
				__assert_error( (nrows-nr-r) < 0 || (ncols-nc-c) < 0, "Invalid dimension\n!"  );

				return __create_viewer( r, c, nr, nc );
			}

			void view( mcore::matrix<T> &m, int r, int c, int nr, int nc ) {

				__assert_error( m.nrows-nr-r < 0 || m.ncols-nc-c < 0, "Invalid dimension\n!" );

				if( !this->is_a_viewer() ) {
					this->clear();
				}
				
				__set_parameters_as_viewer( &m.at(r,c), nr, nc, m.depth, m.stride );

			}

			void clear(void) {
				if( array_ptr != NULL ) {
					delete[] array_ptr;
				} 
				__set_parameters( 0x0, 0x0, 0, 0, 0, 0);
			}

		private:

			/**
			 * @brief Set matrix's parameters as a viewer.
			 * 
			 * @param dt_ptr 
			 * @param nr 
			 * @param nc 
			 * @param ndp 
			 * @param st 
			 */
			void __set_parameters_as_viewer(T * const dt_ptr, int nr, int nc, int ndp, int st)
			{
				__set_parameters(0x0, dt_ptr, nr, nc, ndp, st);
			}

			void __set_parameters(T * const a_ptr, T * const dt_ptr, int nr, int nc, int ndp, int st)
			{
				util::mem::rwp(array_ptr) = a_ptr;
				util::mem::rwp(data) = dt_ptr;
				util::mem::rw(nrows) = nr;
				util::mem::rw(ncols) = nc;
				util::mem::rw(depth) = ndp;
				util::mem::rw(stride) = st;

				util::mem::rw(this->size) = nr*nc*ndp;
			}

			matrix<T> __create_viewer( int row, int col, int nrows, int ncols )
			{
				matrix<T> V;
				V.set_data( &this->at(row,col, 0), nrows, ncols, this->depth, this->stride );
				return V;
			}

	};

} // namespace core

#endif /* _MCORE_MATRIX_HPP_ */
