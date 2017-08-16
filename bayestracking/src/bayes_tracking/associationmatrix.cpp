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
#include "bayes_tracking/associationmatrix.h"

#include "bayes_tracking/BayesFilter/matSup.hpp"
#include <iostream>
#include <float.h>


AssociationMatrix::AssociationMatrix()
{
    RowSize = ColSize = 0;
}


AssociationMatrix::AssociationMatrix(size_t row, size_t col)
{
    setSize(row, col);
}


AssociationMatrix::~AssociationMatrix()
{}


void AssociationMatrix::setSize(size_t row, size_t col)
{
    if ((row > 0) && (col >= 0))
    {
        resize(RowSize = row);
        for (size_t i = 0; i < RowSize; i++) (*this)[i].resize(ColSize = col);
    }
    else
    {
        cerr << "ERROR! - Wrong size for AssociationMatrix (row = " << row
             << ", col = " << col << ")\n";
        exit(1);
    }
}


void AssociationMatrix::print()
{
    for (size_t i = 0; i < RowSize; i++)
    {
        for (size_t j = 0; j < ColSize; j++)
            std::cout << (*this)[i][j] << " ";
        std::cout << std::endl;
    }
}


double AssociationMatrix::mahalanobis(const FM::Vec& v1,
                                      const FM::SymMatrix& R1,
                                      const FM::Vec& v2,
                                      const FM::SymMatrix& R2)
{  // sqrt((v1-v2)' * Si * (v1-v2))
    return mahalanobis(Vec(v1-v2), SymMatrix(R1+R2));
}

Float AssociationMatrix::detS = 0.;

double AssociationMatrix::mahalanobis(const FM::Vec& s,
                                      const FM::SymMatrix& S)
{
    SymMatrix Si(S.size1(), S.size2());
    Float rcond = UdUinversePD(Si, detS, S);  // Si = inv(S)
    Numerical_rcond rclimit;
    rclimit.check_PD(rcond, "S not PD in AssociationMatrix::mahalanobis(...)");
    return sqrt(inner_prod(trans(s), prod(Si, s)));  // sqrt(s' * Si * s)
}


double AssociationMatrix::correlation(const FM::Vec& s, const FM::SymMatrix& S)
{
    SymMatrix Si(S.size1(), S.size2());
    Float rcond = UdUinversePD(Si, detS, S);  // Si = inv(S)
    Numerical_rcond rclimit;
    rclimit.check_PD(rcond, "S not PD in AssociationMatrix::correlation(...)");
    // exp(-0.5 * (s' * Si * s)) / sqrt(2pi^ns * |S|)
    return exp(-0.5*(inner_prod(trans(s),prod(Si, s)))) / sqrt(pow(2*M_PI, (double)s.size())*detS);
}

double AssociationMatrix::correlation_log(const FM::Vec& s, const FM::SymMatrix& S)
{
    SymMatrix Si(S.size1(), S.size2());
    Float rcond = UdUinversePD(Si, detS, S);  // Si = inv(S)
    Numerical_rcond rclimit;
    rclimit.check_PD(rcond, "S not PD in AssociationMatrix::correlation_log(...)");
    // s' * Si * s + ln|S|
    return inner_prod(trans(s), prod(Si, s)) + log(detS);
}


double AssociationMatrix::gate(int dof) {
    switch (dof) {
    case 1: // 1 dof @ P=0.01
        return 2.576;
        break;
    case 2: // 2 dof @ P=0.01
        return 3.035;
        break;
    case 3: // 3 dof @ P=0.01
        return 3.368;
        break;
    case 4: // 4 dof @ P=0.01
        return 3.644;
        break;
    default:
        cerr << "ERROR - undefined value for gate(" << dof << ")\n";
        exit(EXIT_FAILURE);
    }
}


void AssociationMatrix::computeNN(measure_t measure)
{
    NN.clear();
    URow.clear();
    UCol.clear();
    // fill the vectors containing all the rows and columns
    // to be considered for scanning the matrix
    for (size_t i = 0; i < RowSize; i++) URow.push_back(i);
    for (size_t i = 0; i < ColSize; i++) UCol.push_back(i);
    if ((measure == CORRELATION_LOG) || (measure == MAHALANOBIS)) {
        // look minimum distances
        // loop until the set of selected elements covers all the rows or columns
        while (!URow.empty() && !UCol.empty()) {
            double min = DBL_MAX;
            vector< size_t >::iterator min_ri, riEnd = URow.end();
            vector< size_t >::iterator min_ci, ciEnd = UCol.end();
            // find the matrix element with minimum value
            for (vector< size_t >::iterator ri = URow.begin(); ri != riEnd; ri++) {
                for (vector< size_t >::iterator ci = UCol.begin(); ci != ciEnd; ci++) {
                    if ((*this)[*ri][*ci] < min) {
                        // found new minimum, store it with its indexes
                        min = (*this)[*ri][*ci];
                        min_ri = ri;
                        min_ci = ci;
                    }
                }
            }
            // if found a minimum, store its position and remove the relative
            // row and column from further consideration
            if (min < DBL_MAX) {
                match_t p = {*min_ri, *min_ci, min};
                NN.push_back(p);
                URow.erase(min_ri);
                UCol.erase(min_ci);
            }
            else  // if there are no more minimums, stop looking for
                break;
        }
    }
    else if (measure == CORRELATION) {
        // look for maximum probabilities
        // loop until the set of selected elements covers all the rows or columns
        while (!URow.empty() && !UCol.empty()) {
            double max = 0.;
            vector< size_t >::iterator max_ri, riEnd = URow.end();
            vector< size_t >::iterator max_ci, ciEnd = UCol.end();
            // find the matrix element with maximum value
            for (vector< size_t >::iterator ri = URow.begin(); ri != riEnd; ri++) {
                for (vector< size_t >::iterator ci = UCol.begin(); ci != ciEnd; ci++) {
                    if ((*this)[*ri][*ci] > max) {
                        // found new maximum, store it with its indexes
                        max = (*this)[*ri][*ci];
                        max_ri = ri;
                        max_ci = ci;
                    }
                }
            }
            // if found a maximum, store its position and remove the relative
            // row and column from further consideration
            if (max > 0.) {
                match_t p = {*max_ri, *max_ci, max};
                NN.push_back(p);
                URow.erase(max_ri);
                UCol.erase(max_ci);
            }
            else  // if there are no more maximums, stop looking for
                break;
        }
    }
}
