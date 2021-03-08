/** @file matrix.hpp
 *  @brief header file for matrix operations
 *  http://www.vilipetek.com/2013/10/17/polynomial-fitting-in-c-not-using-boost/
*/
#pragma once
#include <vector>
#include <tuple>
#include <stdexcept>
#include <algorithm>
#include <iostream>
/// @brief namespace to hold math algorithms
namespace mathalgo 
{
	using namespace std;
	/// @brief template class to hold matrix functions
	template<class T>
	class matrix  
	{
	public:
		/// constructor
		matrix(unsigned int nRows, unsigned int nCols) : 
			m_nRows( nRows ), 
			m_nCols( nCols ), 
			m_oData( nRows*nCols, 0 )
		{
			if ( !nRows || !nCols )
			{
				throw range_error( "invalid matrix size" );
			}
		}
	
	    	/**  @brief C++ implementation to return identity matrix
		     @param[in] nSize	Size of the identity matrix
		     @return returns square identity matrix of size nSize
	      	*/    
		static matrix identity( unsigned int nSize )
		{
			matrix oResult( nSize, nSize );

			int nCount = 0;
			std::generate( oResult.m_oData.begin(), oResult.m_oData.end(), 
				[&nCount, nSize]() { return !(nCount++%(nSize + 1)); } );

			return oResult;
		}

		/** @brief return the (i,j)th element in the matrix
		 *  @param[in] nRow	i: number of row
		 *  @param[in] nCol	j: number of column
		 *  @return	M(i,j)
		 */ 
		inline T& operator()(unsigned int nRow, unsigned int nCol)
		{
			if ( nRow >= m_nRows || nCol >= m_nCols )
			{
				throw out_of_range( "position out of range" );
			}

			return m_oData[nCol+m_nCols*nRow];
		}

		/** @brief inline function to multiply two matrices
		 *  @param[in] other	The second matrix to multiply
		 *  @return the matrix product
		 */
		inline matrix operator*(matrix& other)
		{
			if ( m_nCols != other.m_nRows )
			{
				throw domain_error( "matrix dimensions are not multiplicable" );
			}

			matrix oResult( m_nRows, other.m_nCols );
			for ( unsigned int r = 0; r < m_nRows; ++r )
			{
				for ( unsigned int ocol = 0; ocol < other.m_nCols; ++ocol )
				{
					for ( unsigned int c = 0; c < m_nCols; ++c )
					{
						oResult(r,ocol) += (*this)(r,c) * other(c,ocol);
					}
				}
			}

			return oResult;
		}

		/// @brief inline function to return the matrix transpose
		inline matrix transpose()
		{
			matrix oResult( m_nCols, m_nRows );
			for ( unsigned int r = 0; r < m_nRows; ++r )
			{
				for ( unsigned int c = 0; c < m_nCols; ++c )
				{
					oResult(c,r) += (*this)(r,c);
				}
			}
			return oResult;
		}

		/// @brief inline function to return the matrix number of rows
		inline unsigned int rows() 
		{
			return m_nRows;
		}

		/// @brief inline function to return the matrix number of columns
		inline unsigned int cols() 
		{
			return m_nCols;
		}
		/// @brief inline function to return the matrix data
		inline vector<T> data()
		{
			return m_oData;
		}
		
	    	/// @brief C++ implementation to print the matrix  
		void print()
		{
			for ( unsigned int r = 0; r < m_nRows; r++ )
			{
				for ( unsigned int c = 0; c < m_nCols; c++ )
				{
					std::cout << (*this)(r,c) << "\t";
				}
				std::cout << std::endl;
			}
		}

	private:
		std::vector<T> m_oData; //!< matrix data in vector format

		unsigned int m_nRows;	//!< number of rows in the matrix
		unsigned int m_nCols;	//!< number of columns in the matrix
	};
};
