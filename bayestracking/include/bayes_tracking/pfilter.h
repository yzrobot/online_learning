/***************************************************************************
 *   Copyright (C) 2006 by Nicola Bellotto   *
 *   nbellotto@lincoln.ac.uk   *
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
#ifndef PFILTER_H
#define PFILTER_H

#include "bayes_tracking/BayesFilter/SIRFlt.hpp"
#include "bayes_tracking/BayesFilter/random.hpp"


using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;

namespace PF
{

class Boost_random : public SIR_random, public Bayesian_filter_test::Boost_random
/*
* Random numbers for SIR from Boost
*/
{
public:
   using Bayesian_filter_test::Boost_random::normal;
   void normal (DenseVec& v)
   {
      Bayesian_filter_test::Boost_random::normal (v);
   }
   using Bayesian_filter_test::Boost_random::uniform_01;
   void uniform_01 (DenseVec& v)
   {
      Bayesian_filter_test::Boost_random::uniform_01 (v);
   }
};

}  // namespace


/**
@author Nicola Bellotto
*/
class PFilter : public SIR_kalman_scheme
{
public:
   /**
    * Constructor - include initialization with null state and null covariance
    * @param x_size State size
    * @param s_size Number of samples
    * @param random_helper Random generator
    * @return 
    */
   PFilter(std::size_t x_size,
           std::size_t s_size,
           SIR_random &random_helper);

   /**
    * Constructor
    * @param x_size State size
    * @param s_size Number of samples (default 1000)
    */
   PFilter(std::size_t x_size,
           std::size_t s_size = 1000);

   /**
    * Constructor
    * @param x0 State vector
    * @param P0 Covariance matrix
    * @param s_size Number of samples
    * @param random_helper Random generator
    */
   PFilter(const FM::Vec& x0,
           const FM::SymMatrix& P0,
           std::size_t s_size,
           SIR_random& random_helper);
   
   /**
    * Constructor
    * @param x0 State vector
    * @param P0 Covariance matrix
    * @param s_size Number of samples (default 1000)
    */
   PFilter(const FM::Vec& x0,
           const FM::SymMatrix& P0,
           std::size_t s_size = 1000);
   
   /**
    * Destructor
    */
   ~PFilter();
   
   PFilter& operator= (const PFilter& a)
   {
      SIR_scheme::operator=(a);
      x = a.x;
      X = a.X;
      return *this;
   }
   
   /**
    * Initialize state and covariance
    * @param x0 State vector
    * @param P0 Covariance matrix
    */
   void init(const FM::Vec& x0, const FM::SymMatrix& P0);

   /**
    * Prediction the state
    * @param predict_model Prediction model
    */
   void predict(Sampled_predict_model& predict_model);
   
   /**
    * Predict the observation from the last predicted state and the covariance R_p of the predicted observation
    * @param observe_model Observation model
    * @param z_pred Predicted observation (return)
    * @param R_pred Predicted observation covariance (return)
    */
   void predict_observation(Correlated_additive_observe_model& observe_model,
                            FM::Vec& z_pred, FM::SymMatrix& R_pred);
                           
  /**
    * Perform correction of the predicted state according to the observation.
    * @param observe_model Observation model
    * @param z Current observation
    */
   void observe(Likelihood_observe_model& observe_model,
                const FM::Vec& z);

//    void roughen()
//    // Generalised roughening:  Default to roughen_minmax
//    {
//       if (rougheningK != 0)
//          roughen_minmax (S, rougheningK);
//    }
   
   /**
    * Return the logarithm of the likelihhod for the last observation
    * @return Logarithm of the likelihood
    */
   double logLikelihood();

public:
   /** Particle weights */
   FM::DenseVec& w;

private:
   std::size_t x_size;
   PF::Boost_random rnd;
   double m_likelihood;
};

#endif
