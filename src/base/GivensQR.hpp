/** @file GivensQR.hpp
 *  @brief header file for Givens QR functions
 *  http://www.vilipetek.com/2013/10/17/polynomial-fitting-in-c-not-using-boost/
*/
#pragma once
#include "matrix.hpp"
#include <stdexcept>
#include <math.h>
#include <algorithm>

/// @brief namespace to hold math algorithms
namespace mathalgo
{
	using namespace std;
	/// @brief template class Givens
	template<typename T>
	class Givens
	{
	public:
		Givens() : m_oJ(2,2), m_oQ(1,1), m_oR(1,1)
		{
		}

		/** @brief Calculate the inverse of a matrix using the QR decomposition.
		 *  @param[in] oMatrix		matrix to inverse
		 *  @return the matrix inverse
		*/
		const matrix<T> Inverse( matrix<T>& oMatrix )
		{
			if ( oMatrix.cols() != oMatrix.rows() )
			{
				throw domain_error( "matrix has to be square" );
			}
			matrix<T> oIdentity = matrix<T>::identity( oMatrix.rows() );
			Decompose( oMatrix );
			return Solve( oIdentity );
		}
		
		/** @brief Performs QR factorization using Givens rotations.
		 *  @param[in] oMatrix		input matrix to decompose
		 *  @return the decomposed matrix
		*/
		void Decompose( matrix<T>& oMatrix )
		{
			int nRows = oMatrix.rows();
			int nCols = oMatrix.cols();


			if ( nRows == nCols )
			{
				nCols--;
			}
			else if ( nRows < nCols )
			{
				nCols = nRows - 1;
			}

			m_oQ = matrix<T>::identity(nRows);
			m_oR = oMatrix;

			for ( int j = 0; j < nCols; j++ )
			{
				for ( int i = j + 1; i < nRows; i++ )
				{
					GivensRotation( m_oR(j,j), m_oR(i,j) );
					PreMultiplyGivens( m_oR, j, i );
					PreMultiplyGivens( m_oQ, j, i );
				}
			}

			m_oQ = m_oQ.transpose();
		}
		
		/** @brief Find the solution for a matrix.
			http://en.wikipedia.org/wiki/QR_decomposition#Using_for_solution_to_linear_inverse_problems
		 *  @param[in] oMatrix		input matrix to solve
		 *  @return the solved matrix
		 */
		matrix<T> Solve( matrix<T>& oMatrix )
		{
			matrix<T> oQtM( m_oQ.transpose() * oMatrix );
			int nCols = m_oR.cols();
			matrix<T> oS( 1, nCols );
			for (int i = nCols-1; i >= 0; i-- )
			{
				oS(0,i) = oQtM(i, 0);
				for ( int j = i + 1; j < nCols; j++ )
				{
					oS(0,i) -= oS(0,j) * m_oR(i, j);
				}
				oS(0,i) /= m_oR(i, i);
			}

			return oS;
		}
		/// @brief returns the Q matrix
		const matrix<T>& GetQ()
		{
			return m_oQ;
		}
		/// @brief returns the R matrix
		const matrix<T>& GetR()
		{
			return m_oR;
		}

	private:
		/** @brief Givens rotation is a rotation in the plane spanned by two coordinates axes.
			http://en.wikipedia.org/wiki/Givens_rotation
		 *  @param[in] a	The first axis
		 *  @param[in] b	The second axis
		 */
		void GivensRotation( T a, T b )
		{
			T t,s,c;
			if (b == 0)
			{
				c = (a >=0)?1:-1;
				s = 0; 
			}
			else if (a == 0)
			{
				c = 0;
				s = (b >=0)?-1:1;
			}
			else if (abs(b) > abs(a))
			{
				t = a/b;
				s = -1/sqrt(1+t*t);
				c = -s*t;
			}
			else
			{
				t = b/a;
				c = 1/sqrt(1+t*t);
				s = -c*t;
			}
			m_oJ(0,0) = c; m_oJ(0,1) = -s;
			m_oJ(1,0) = s; m_oJ(1,1) = c;
		}

		/** @brief Get the premultiplication of a given matrix by the Givens rotation.
		 *  @param[in,out]	oMatrix		The matrix
		 *  @param[in] 		i		the i value
		 *  @param[in] 		j		the j value
		*/
		void PreMultiplyGivens( matrix<T>& oMatrix, int i, int j )
		{
			int nRowSize = oMatrix.cols();

			for ( int nRow = 0; nRow < nRowSize; nRow++ )
			{
				double nTemp = oMatrix(i,nRow) * m_oJ(0,0) + oMatrix(j,nRow) * m_oJ(0,1);
				oMatrix(j,nRow) = oMatrix(i,nRow) * m_oJ(1,0) + oMatrix(j,nRow) * m_oJ(1,1);
				oMatrix(i,nRow) = nTemp;
			}
		}

	private:
		matrix<T> m_oQ; //!< Q matrix
		matrix<T> m_oR; //!< R matrix
		matrix<T> m_oJ; //!< J matrix
	};
}
