/***************************************************************************
 *   Copyright (C) 2011 by Nicola Bellotto                                 *
 *   nbellotto@lincoln.ac.uk                                               *
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
#ifndef JPDA_H
#define JPDA_H


#include <vector>
#include <iostream>
#include "bayes_tracking/BayesFilter/bayesFlt.hpp"
#include "bayes_tracking/BayesFilter/matSup.hpp"
#include <float.h>


using namespace std;
using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;

namespace jpda
{

double Gauss(const Vec& s, const SymMatrix& S) {
   SymMatrix Si(S.size1(), S.size2());
   Float detS;
   Float rcond = UdUinversePD(Si, detS, S);  // Si = inv(S)
   Numerical_rcond rclimit;
   rclimit.check_PD(rcond, "S not PD");
   // exp(-0.5 * (s' * Si * s)) / sqrt(2pi^ns * |S|)
   return exp(-0.5*(inner_prod(trans(s),prod(Si, s)))) / sqrt(pow(2*M_PI, (double)s.size())*detS);
}

double logGauss(const Vec& s, const SymMatrix& S) {
   SymMatrix Si(S.size1(), S.size2());
   Float detS;
   Float rcond = UdUinversePD(Si, detS, S);  // Si = inv(S)
   Numerical_rcond rclimit;
   rclimit.check_PD(rcond, "S not PD");
   // log(exp(-0.5 * (s' * Si * s)) / sqrt(2pi^ns * |S|))
   return -0.5*(inner_prod(trans(s),prod(Si, s))) - 0.5*((double)s.size()*(log(2*M_PI))+log(detS));
}


enum EmptyTag {Empty};  // Tag type used for empty matrix constructor

/** Matrix class */
template < typename Element >
class Matrix : public vector< vector< Element > > {
private:
   size_t m_rows;
   size_t m_cols;
public:
   Matrix(EmptyTag) {
      m_rows = 0;
      m_cols = 0;
   }

   Matrix(size_t rows, size_t cols) {
      resize(rows, cols);
   }

   void resize(size_t rows, size_t cols) {
      vector< vector< Element > >::resize(rows);
      for (size_t i = 0; i < rows; i++){
         (*this)[i].resize(cols);
      }
      m_rows = rows;
      m_cols = cols;
   }

   void set(Element e) {
      for (size_t i = 0; i < m_rows; i++) {
         for (size_t j = 0; j < m_cols; j++) {
            (*this)[i][j] = e;
         }
      }
   }

   size_t getRows() { return m_rows; }
   
   size_t getCols() { return m_cols; }

   void print() {
      for (size_t i = 0; i < m_rows; i++) {
         for (size_t j = 0; j < m_cols; j++) {
            cout << (*this)[i][j] << " ";
         }
         cout << endl;
      }
   }
};



/** MultiMatrix class */
template < typename Element >
class MultiMatrix : public vector< MultiMatrix< Element > > {
private:
   size_t m_dim;
public:
   Element val;
   
   MultiMatrix(EmptyTag) {
      m_dim = 0;
      val = Element();
   }

   MultiMatrix(const MultiMatrix& mm) {
      m_dim = mm.m_dim;
      val = mm.val;
      if (m_dim) {
         size_t s = mm.size();
         for (size_t i = 0; i < s; i++) {
            (*this).push_back(mm[i]);
         }
      }
   }
   
   MultiMatrix(const vector< size_t >& sizes) {
      init(sizes);
   }
   
   void init(const vector< size_t >& sizes) {
      size_t s = sizes.size();
      assert(s);
      m_dim = s;
      val = Element();
      
      (*this).clear();
      size_t extent = sizes[0];
      assert(extent);
      
      if (s > 1) {
         vector< size_t > sub(sizes.begin()+1, sizes.end());
         MultiMatrix< Element > m(sub);
         for (size_t i = 0; i < extent; i++) {
            this->push_back(m);
         }
      }
      else {
         MultiMatrix< Element > m(Empty);
         for (size_t i = 0; i < extent; i++) {
            this->push_back(m);
         }
      }
   }

    Element& operator()(const vector< size_t >& pos) {
    size_t s = pos.size();
    assert(s == m_dim);              // check valid number of coordinates
    if (s > 0) {
      assert(pos[0] < (*this).size()); // check coordinate within max range
        vector< size_t > sub(pos.begin() + 1, pos.end());
        return (*this)[pos[0]](sub);
    }
    return /*(*this)[pos[0]].*/val;
    }

   void getSize(vector< size_t >& sizes) {
      if ((*this).size()) {
         sizes.push_back((*this).size());
         (*this)[0].getSize(sizes);
      }
   }

   vector< size_t > getSize() {
      vector< size_t > s;
      getSize(s);
      return s;
   }

   void print() {
      string str;
      print(str);
   }

   void print(const string& str) {
      if ((*this).size()) {
         size_t length = (*this).size();
         for (size_t i = 0; i < length; i++) {
            ostringstream os;
            os << "[" << i << "]";
            (*this)[i].print(string(str + os.str()));
         }
      }
      else {
         cout << str << " = " << val << endl;
      }
   }
};


/** Association structure */
typedef struct {
   size_t z;
   size_t t;
} association_t;

/** Association vector class */
class Association : public vector< association_t > {
public:
   void print() {
      iterator it, itEnd = end();
      for (it = begin(); it != itEnd; it++) {
         cout << "(" << it->z << "," << it->t << ") ";
      }
      cout << endl;
   }
};

/**
 *	@author Nicola Bellotto <nbello@essex.ac.uk>
 */
class JPDA {
public:
   /**
    * Constructor
    * @param zNum Vector with number of observations per each sensor
    * @param tNum Number of targets
    */
   JPDA(const vector< size_t >& zNum, size_t tNum) : m_beta(Empty) {
      init(zNum, tNum);
   }

   /**
    * Destructor
    */

   ~JPDA() {
   }


   /**
    * Reset 
    * @param zNum Vector with number of observations per each sensor
    * @param tNum Number of targets
    */
   void init(const vector< size_t >& zNum, size_t tNum) {
      m_tNum = tNum;
      m_mNum = zNum.size();

      Matrix< bool > omega(Empty);
      Matrix< double > mat(Empty);
      Omega.clear();
      Lambda.clear();
      Beta.clear();
      logP.resize(m_mNum);
      
      vector< size_t > betaSizes;
      betaSizes.push_back(m_tNum+1);   // include clutter t0
      vector< size_t >::const_iterator zi, ziEnd = zNum.end();
      for (zi = zNum.begin(); zi != ziEnd; zi++) {
         betaSizes.push_back((*zi)+1); // include no measure z0
      }
      m_beta.init(betaSizes);
      
      for (size_t m = 0; m < m_mNum; m++) {
         assert(zNum[m]);
         omega.resize(zNum[m], tNum+1);
         // insert 1 on the first column
         for (size_t i = 0; i < zNum[m]; i++) {
            omega[i][0] = true;
         }
         Omega.push_back(omega);
         mat.resize(zNum[m], tNum+1);  // include clutter y0 column
         Lambda.push_back(mat);
         mat.resize(zNum[m]+1, tNum+1);   // include no measure z0 row
         Beta.push_back(mat);
      }
      m_bestXsi = 0;
   }

   /**
    * Calculate all the feasible associations
    * @return Number of associations
    */
   int getAssociations() {
      vector< size_t> colVec;
      for (size_t j = 0; j < m_tNum+1; j++) {
         colVec.push_back(j);
      }

      Association assoc;

      for (size_t m = 0; m < m_mNum; m++) {
         m_assocVec.clear();
         assoc.clear();
         getAssociation(0, colVec, assoc, m);
         m_xsi.push_back(m_assocVec);
      }

      m_assocVec.clear();
      Xsi.clear();
      fill(0, m_mNum);
      
      return Xsi.size();
   }

   /**
    * Calculate the probabilities of all the feasible associations
    * @param Pd Target detection probability (default 0.8)
    * @param C Density of false measurements (default 0.2)
    */
   void getProbabilities(double Pd = 0.8, double C = 0.2) {
      double logC = log(C);
      double logPd = log(Pd);
      double log1_Pd = log(1-Pd);
      // events probabilities
      for (size_t m = 0; m < m_mNum; m++) {  // for each sensor
         logP[m].clear();
         double sum = 0;
         vector< vector< Association > >::iterator vi, viEnd = Xsi.end();
         // compute probability of each feasible association matrix
         for (vi = Xsi.begin(); vi != viEnd; vi++) {
            double logp = 0;
            size_t detected = 0; // counter of detected target
            size_t clutters = 0; // counter of false measurements
            Association::iterator ai, aiEnd = (*vi)[m].end();
            for (ai = (*vi)[m].begin(); ai != aiEnd; ai++) { // for each association <z,t>
                if (ai->t) {   // avoid t=0 (clutter case)
                  logp += Lambda[m][ai->z][ai->t];
                  detected++;
                }
                else {
                  clutters++;
                }
            }
            logp += clutters * logC;                  // density of false measurements
            logp += detected * logPd;                 // tot detection probability
            logp += (m_tNum - detected) * log1_Pd;    // tot non-detection probability
            // store event probability (single sensor)
            logP[m].push_back(logp);
            sum += exp(logp);
         }
         // normalization of event probabilities (single sensor)
         double logSum = log(sum);
         vector< double >::iterator pi, piEnd = logP[m].end();
         for (pi = logP[m].begin(); pi != piEnd; pi++) {
            *pi -= logSum;
         }
         // association probabilities (single sensor)
         size_t j, jEnd = Xsi.size();
         for (j = 0; j < jEnd; j++) {
            Association::iterator ai, aiEnd = Xsi[j][m].end();
            for (ai = Xsi[j][m].begin(); ai != aiEnd; ai++) {
               Beta[m][ai->z+1][ai->t] += exp(logP[m][j]);
            }
         }
         // compute beta0
         for (size_t t = 1; t <= m_tNum; t++) {
            size_t rows = Beta[m].getRows();
            double betaSum = 0;
            for (size_t j = 1; j < rows; j++) {
               betaSum += Beta[m][j][t];
            }
            Beta[m][0][t] = 1.0 - betaSum;
         }
      }
      // calculate multisensor association probabilities
      for (size_t t = 1; t <= m_tNum; t++) {
         vector< size_t > l(m_beta[t].getSize());
         vector< size_t > pos(m_mNum);
         size_t last = pos.size()-1;
         while (pos[last] < l[last]) {    // scan all the m_beta[t] elements
            m_beta[t](pos) = 1.0;
            for (size_t m = 0; m < m_mNum; m++) {
               m_beta[t](pos) *= Beta[m][pos[m]][t];
            }  // current element update finished
            // update position vector
            pos[0]++;
            for (size_t i = 0; i < last; i++) {
               if (pos[i] == l[i]) {   // max coordinated reached
                  pos[i] = 0;
                  pos[i+1]++;
               }
               else {   // no need to go further
                  break;
               }
            }
         }
      }
   }

   /**
    * Compute Multisensor NNJPDA and return the resulting vector of associations, for each sensor
    * @param association Vector of associations
    * @param minP Minimum association probability (default 0.1)
    */
   void getMultiNNJPDA(vector< Association >& association, double minP = 0.1) {
      assert(association.size() == m_mNum);
      bool done = false;
      vector< size_t > l(m_beta.getSize());
      size_t dims = l.size();
      vector< vector< size_t > > planes(dims);
      for (size_t i = 0; i < dims; i++) {
         size_t s = l[i];
         for (size_t j = 0; j < s; j++) {
            planes[i].push_back(j);
         }
      }
      // retrieve associations with maximum association probabilities
      while (!done) {
         double max = minP;
         vector< size_t > pos(dims);
         vector< size_t > maxPos(pos);
         vector< vector< size_t >::iterator > pit(dims);
         for (size_t i = 0; i < dims; i++) { // start at the beginning of each plane
            pit[i] = planes[i].begin();
            pos[i] = *pit[i];
         }
         vector< vector< size_t >::iterator > maxPit(pit);
         size_t last = dims-1;
         // scan the matrix for the max association probability
         while (pit[last] != planes[last].end()) {
            // get position vector
            for (size_t i = 0; i <= last; i++) {
               pos[i] = *pit[i];
            }
            // check for maximum
            if (m_beta(pos) > max) {
               max = m_beta(pos);
               maxPos = pos;
               maxPit = pit;
            }
            // update position vector
            pit[0]++;
            for (size_t i = 0; i < last; i++) {
               if (pit[i] == planes[i].end()) {   // max coordinated reached
                  pit[i] = planes[i].begin();
                  pit[i+1]++;
               }
               else {   // no need to go further
                  break;
               }
            }
         }
         association_t a;
         if ((a.t = maxPos[0])) {
            planes[0].erase(maxPit[0]);   // remove target plane
         }
         if (planes[0].size() < 2) {   // only clutter plane left
            done = true;               // so it is finished!
         }
         bool allEmpty = true;
         for (size_t m = 0; m < m_mNum; m++) {
            if (maxPos[m+1]) {   // it's a proper measurement
               a.z = maxPos[m+1] - 1;  // fix within range 0...(m-1)
               association[m].push_back(a);  // store association
               planes[m+1].erase(maxPit[m+1]);  // remove plane from further consideration
            }
            if (planes[m+1].size() > 1) {   // more measurements planes left
               allEmpty = false;              // so not finished yet
            }
         }
         if (done |= allEmpty) { // if finished, throw out the remaining measurements as clutters
            for (size_t m = 0; m < m_mNum; m++) {  
               if (planes[m+1].size() > 1) {  // there's something left
                  vector< size_t >::iterator it, itEnd = planes[m+1].end();
                  for (it = planes[m+1].begin() + 1; it != itEnd; it++) {
                     a.t = 0;
                     a.z = *it - 1;  // fix within range 0...(m-1)
                     association[m].push_back(a);  // store association
                  }
               }
            }
         }
      }
   }
   
   /**
    * Compute NNJPDA (independent sensors) and return the resulting vector of associations, for each sensor
    * @param association Vector of associations
    * @param minP Minimum association probability (default 0.1)
    */
   void getMonoNNJPDA(vector< Association >& association, double minP = 0.1) {
      vector< size_t > rows;
      vector< size_t > cols;

      assert(association.size() == m_mNum);
      
      for (size_t m = 0; m < m_mNum; m++) {
         // fill the vectors containing all the rows and columns
         // to be considered for scanning the matrix
         rows.clear();
         cols.clear();
         size_t zNum = Beta[m].size(); // number of readings varies with sensor
         for (size_t i = 0; i < zNum; i++) rows.push_back(i);
         for (size_t j = 0; j < m_tNum+1; j++) cols.push_back(j); // avoid 0 column
         // look for maximum probabilities
         // loop until the set of selected elements covers all the rows or columns
         while (rows.size() > 1 || rows[0] > 0) {
            double max = -1;
            vector< size_t >::iterator ri, max_ri, riEnd = rows.end();
            vector< size_t >::iterator ci, max_ci, ciEnd = cols.end();
            // find the matrix element with maximum probability
            for (ri = rows.begin(); ri != riEnd; ri++) {
               for (ci = cols.begin(); ci != ciEnd; ci++) {
                  if (Beta[m][*ri][*ci] > max) {
                     // found new minimum, store it with its indexes
                     max = Beta[m][*ri][*ci];
                     max_ri = ri;
                     max_ci = ci;
                  }
               }
            }
            // if found a maximum, store its position and remove the relative
            // row and column from further consideration (except column of target 0 - clutter)
            association_t a;
            if (*max_ri) {  // it's a proper measurement
              if (max > minP) {
                a.z = *max_ri - 1; // fix within range 0...(m-1)
                a.t = *max_ci;
              }
              else {   // clutter
                a.z = *max_ri - 1; // fix within range 0...(m-1)
                a.t = 0;
              }
              association[m].push_back(a);
              rows.erase(max_ri);
            }
            if (*max_ci) { // don't erase column 0
               cols.erase(max_ci);
            }
         }
      }
   }

   /**
    * Get the feasible association with highest probability
    * @return Index of feasible association Xsi
    */
   size_t getBestXsi() {
      return m_bestXsi;
   }

private:
   void getAssociation(size_t row, vector< size_t> colVec, Association& assoc, size_t m) {
      vector< size_t>::iterator ci;
      for (ci = colVec.begin(); ci != colVec.end(); ci++) {
         if (Omega[m][row][*ci]) {
            association_t a = {row, *ci};
            assoc.push_back(a);
            if (row >= (Omega[m].getRows() - 1)) {
               m_assocVec.push_back(assoc);
               assoc.pop_back();
            }
            else {
               vector< size_t > nextColVec = colVec;
               if (*ci != 0) {
                  nextColVec.erase(nextColVec.begin() + (ci - colVec.begin()));
               }
               getAssociation(row + 1, nextColVec, assoc, m);
            }
         }
      }
      assoc.pop_back();
   }
   
   void fill(size_t start, size_t end) {
      if (start < end) {
         vector< Association >::iterator vi, viEnd = m_xsi[start].end();
         for (vi = m_xsi[start].begin(); vi != viEnd; vi++) {
            m_assocVec.push_back(*vi);
            fill(start+1, end);
            m_assocVec.pop_back();
         }
      }
      else {
         vector< Matrix< bool > > av;
         vector< Association >::iterator vi, viEnd = m_assocVec.end();
         for (vi = m_assocVec.begin(); vi != viEnd; vi++) {
            Matrix< bool > a(vi->size(), m_tNum);
            Association::iterator ai, aiEnd = vi->end();
            for (ai = vi->begin(); ai != aiEnd; ai++) {
               a[ai->z][ai->t] = true;
            }
            av.push_back(a);
         }
         Xsi.push_back(m_assocVec);
      }
   }

   size_t getTargetAssociations(Association& assoc, size_t target) {
      size_t num = 0;
      Association::iterator ai, aiEnd = assoc.end();
      for (ai = assoc.begin(); ai != aiEnd; ai++) {
         if (ai->t == target) {
            num++;
         }
      }
      return num;
   }

public:
   /** Vector of validation matrices, one for each sensor */
   vector< Matrix< bool > > Omega;
   /** Vector of validation matrices, one for each sensor */
   vector< Matrix< double > > Lambda;
   /** Vector of association matrices, one for each sensor */
   vector< Matrix< double > > Beta;
   /** Vector of feasible matrices, one for each sensor */
   vector< vector< Association > > Xsi;
   /** Vector of association's (log) probabilities */
   vector< vector < double > > logP;

private:
   size_t m_tNum;
   size_t m_mNum;
   vector< Association > m_assocVec;
   vector< vector< Association > > m_xsi;
   size_t m_bestXsi;
   MultiMatrix< double > m_beta;
};

}  // namespace jpda

#endif
