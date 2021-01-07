/***************************************************************************
 *   Copyright (C) 2011 by Nicola Bellotto                                 *
 *   nbellotto@lincoln.ac.uk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef ASSOCIATIONMATRIX_H
#define ASSOCIATIONMATRIX_H

#include <vector>
#include "bayes_tracking/BayesFilter/bayesFlt.hpp"


using namespace std;
using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;

typedef enum{CORRELATION, MAHALANOBIS, CORRELATION_LOG} measure_t;


/**
Association matrix

@author Nicola Bellotto
*/
class AssociationMatrix : public std::vector< std::vector<double> >
{
public:
    /**
     * Constructor
     */
    AssociationMatrix();
    
    /**
     * Constructor
     * @param row Number of rows
     * @param col Number of columns
     */
    AssociationMatrix(size_t row, size_t col);

    /**
     * Destructor
     * @return 
     */
    ~AssociationMatrix();

    /**
     * Set matrix size
     * @param row Number of rows
     * @param col Number of columns
     */
    void setSize(size_t row, size_t col);

   /**
    * Print matrix elements on standard output
    */
   void print();

   /**
    * Get the number of rows
    * @return Number of rows
    */
   inline size_t getRowSize() { return RowSize; }

   /**
    * Get the number of columns
    * @return Number of columns
    */
   inline size_t getColSize() { return ColSize; }

   /**
    * Calculate the Mahalanobis distance:
    * d = sqrt((v1-v2)' * inv(R1+R2) * (v1-v2))
    * @param v1 First vector
    * @param R1 Covariance matrix of v1
    * @param v2 Second vector
    * @param R2 Covariance matrix of v2
    * @return Mahalanobis distance
    */
   static double mahalanobis(const FM::Vec& v1,
                             const FM::SymMatrix& R1,
                             const FM::Vec& v2,
                             const FM::SymMatrix& R2);

   /**
    * Calculate the Mahalanobis distance:
    * d = sqrt(s' * inv(S) * s)
    * @param s Innovation vector
    * @param S Covariance matrix of s
    * @return Mahalanobis distance
    */
   static double mahalanobis(const FM::Vec& s,
                             const FM::SymMatrix& S);
   
   /**
    * Calculate the correlation of the innovation:
    * c = exp(-0.5 * d^2) / sqrt(2pi * |S|)
    * where @p d is the Mahalanobis distance
    * @param s Innovation vector
    * @param S Covariance matrix of s
    * @return Correlation
    */
   static double correlation(const FM::Vec& s,
                             const FM::SymMatrix& S);
   
   /**
    * Calculate distance
    * d = s' * Si * s + ln|S|
    * See: Bogler Philip. Radar Principles with Applications to Tracking Systems, John Wiley & Sons, 1989.
    * @param s Innovation vector
    * @param S Covariance matrix of s
    * @return Distance
    */
   static double correlation_log(const FM::Vec& s, const FM::SymMatrix& S);

   /**
    * Return value of gate from Chi-square distribution
    * @param dof Degree of freedom (size of the vector to be gated)
    * @return Gate value (square root of relative value in table of Chi-square distribution for P = 0.01)
    */
   static double gate(int dof);
   
   /**
    * Compute Nearest Neighbour (NN) assignment on the association matrix.
    * The result is a vector of (row,col) pairs stored in @p NN, where the first
    * element corresponds to the pair with the best similarity measure
    * @param measure Association measure: 'CORRELATION_LOG' (default), 'CORRELATION' or 'MAHALANOBIS'
    */
   void computeNN(measure_t measure);

public:
   typedef struct { size_t row; size_t col; double match; } match_t;
   /**
    * Vector of pairs (row,col) and relative similarity match value
    * computed with @p computeNN(...)
    */
   std::vector< match_t > NN;
   
   /**
    * Vector of unmatched rows indexes
    */
   std::vector< size_t > URow;

   /**
    * Vector of unmatched columns indexes
    */
   std::vector< size_t > UCol;

private:
   size_t RowSize;   // matrix row size
   size_t ColSize;   // matrix column size
   static Float detS;
};

// short name version
typedef AssociationMatrix AM;


#endif
