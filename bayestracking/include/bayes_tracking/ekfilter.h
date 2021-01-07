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
#ifndef EKFILTER_H
#define EKFILTER_H

#include <vector>
#include "bayes_tracking/BayesFilter/covFlt.hpp"


using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;

/**
EKF state estimator

@author Nicola Bellotto
*/
class EKFilter : public Covariance_scheme
{
public:
   /**
    * Constructor - include initialization with null state and null covariance
    */
   EKFilter(std::size_t x_size);

   /**
    * Constructor
    * @param x0 State vector
    * @param P0 Covariance matrix
    */
   EKFilter(const FM::Vec& x0, const FM::SymMatrix& P0);
   
   /**
    * Destructor
    */
   ~EKFilter();
   
   /**
    * Initialize state and covariance
    * @param x0 State vector
    * @param P0 Covariance matrix
    */
   void init(const FM::Vec& x0, const FM::SymMatrix& P0);

   /**
    * Prediction (include Jacobian update)
    * @param f Linearized model
    */
   Bayes_base::Float predict (Linrz_predict_model& f);
   
   /**
    * Prediction (include Jacobian update)
    * @param f Gaussian model
    */
   Bayes_base::Float predict (Gaussian_predict_model& f);
   
   /**
    * Perform prediction and correction to update the state
    * @param predict_model
    * @param observe_model 
    * @param z 
    */
   void update(Linrz_predict_model& predict_model,
               Linrz_correlated_observe_model& observe_model,
               const FM::Vec& z);

   /**
    * Predict the observation from the last predicted state and the covariance R_p of the predicted observation
    * @param observe_model Observation model
    * @param z_pred Predicted observation (return)
    * @param R_pred Predicted observation covariance (return)
    */
   void predict_observation(Linrz_correlated_observe_model& observe_model,
                           FM::Vec& z_pred, FM::SymMatrix& R_pred);
                           
   /**
    * Update the state from innovation and relative covariance.
    * @param h Observation model
    * @param si Innovation
    * @param Si Modified innovation covariance
    */
   Bayes_base::Float observeInnovation(Linrz_correlated_observe_model& h, const Bayesian_filter_matrix::Vec& si, const Bayesian_filter_matrix::SymMatrix& Si);
 
   /** Override method */
   Bayes_base::Float observe (Linrz_correlated_observe_model& h, const FM::Vec& z);
   /** Override method */
   Bayes_base::Float observe (Linrz_uncorrelated_observe_model& h, const FM::Vec& z);
   
   /**
    * Return the logarithm of the (normalized) likelihhod after the last observation
    * @return Logarithm of the likelihood
    */
   double logLikelihood();

public:
   /** Innovation */
   FM::Vec s;
   /** Predicted observation */
   FM::Vec z_p;

private:
   std::size_t x_size;
   std::size_t X_size;
};

#endif
