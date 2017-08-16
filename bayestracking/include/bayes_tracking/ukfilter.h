/***************************************************************************
 *   Copyright (C) 2006 by Nicola Bellotto                                 *
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
#ifndef UKFILTER_H
#define UKFILTER_H

#include <vector>
#include "bayes_tracking/BayesFilter/unsFlt.hpp"


using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;

/**
UKF state estimator

@author Nicola Bellotto
*/
class UKFilter : public Unscented_scheme
{
public:
   /**
    * Constructor - include initialization with null state and null covariance
    */
   UKFilter(std::size_t x_size);

   /**
    * Constructor
    * @param x0 State vector
    * @param P0 Covariance matrix
    */
   UKFilter(const FM::Vec& x0, const FM::SymMatrix& P0);
   
   /**
    * Destructor
    */
   ~UKFilter();
   
   /**
    * Initialize state and covariance
    * @param x0 State vector
    * @param P0 Covariance matrix
    */
   void init(const FM::Vec& x0, const FM::SymMatrix& P0);

   /**
    * Perform prediction and correction to update the state
    * @param predict_model
    * @param observe_model 
    * @param z 
    */
   void update(Additive_predict_model& predict_model,
               Correlated_additive_observe_model& observe_model,
               const FM::Vec& z);

   /**
    * Predict the observation from the last predicted state and the covariance R_p of the predicted observation
    * @param observe_model Observation model
    * @param z_pred Predicted observation (return)
    * @param R_pred Predicted observation covariance (return)
    */
   void predict_observation(Correlated_additive_observe_model& observe_model,
                            FM::Vec& z_pred, FM::SymMatrix& R_pred);
                           
   /**
    * Update the state from innovation and relative covariance.
    * @param h Observation model
    * @param si Innovation
    * @param Si Modified innovation covariance
    */
   Bayes_base::Float observeInnovation(Correlated_additive_observe_model& h,
      const Vec& si, const SymMatrix& Si);
      
   /**
    * Return the logarithm of the (normalized) likelihhod after the last observation
    * @return Logarithm of the likelihood
    */
   double logLikelihood();

public:
   /** Predicted observation */
   FM::Vec z_p;

private:
   void unscented(FM::ColMatrix& XX,
                  const FM::Vec& x,
                  const FM::SymMatrix& X,
                  Float scale);

private:
   std::size_t x_size;
   std::size_t XX_size;
};

#endif
