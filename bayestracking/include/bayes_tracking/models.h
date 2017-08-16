/***************************************************************************
 *   Copyright (C) 2006 by Nicola Bellotto                                 *
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

#ifndef MODELS_H
#define MODELS_H


#include "bayes_tracking/BayesFilter/covFlt.hpp"
#include "bayes_tracking/BayesFilter/SIRFlt.hpp"
#include "bayes_tracking/BayesFilter/models.hpp"
#include "bayes_tracking/BayesFilter/random.hpp"
#include "bayes_tracking/jacobianmodel.h"

using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;
using namespace Bayesian_filter_test;

namespace Models
{

// Delta of Dirac
template <class scalar>
inline scalar deltad(scalar x)
{
   return x == 0. ? 1. : 0.;
}

// Square
template <class scalar>
inline scalar sqr(scalar x) {
  return x*x;
}

/**
 * Random numbers for SIR from Boost
 */
class Boost_random : public SIR_random, public Bayesian_filter_test::Boost_random
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

    //==========================================================================
    //============================== 2D CVModel ================================
    //==========================================================================

/**
 * Constant velocity prediction model for linear human motion
 * Noises are modelled as accelerations
 */
class CVModel :
  public JacobianModel,
  public Linrz_predict_model,
  public Sampled_predict_model
{
public:
  /** Size of the human state */
  static const std::size_t x_size = 4;
  /** Size of the human state noise */
  static const std::size_t q_size = 2;
  mutable FM::Vec fx;
  double dt;

  /**
  * Contructor
  * @param wxSD Standard deviation of noise_x
  * @param wySD Standard deviation of noise_y
  */
  CVModel(Float wxSD, Float wySD);

  /**
  * Definition of sampler for additive noise model given state x
  *  Generate Gaussian correlated samples
  * Precond: init_GqG, automatic on first use
  */
  virtual const FM::Vec& fw(const FM::Vec& x) const;

  /** initialise predict given a change to q,G
  *  Implementation: Update rootq
  */
  void init_GqG() const;

  /**
  * Non-linear prediction model
  * @param x State vector x(k-1)
  * @return A-priori estimation x^(k)
  */
  const FM::Vec& f(const FM::Vec& x) const;

  /**
  * Model update, to be called before prediction
  * @param dt Time interval since last estimation
  */
  void update(double dt);

  /**
  * Model update, to be called before prediction
  * @param x State vector around which the linearization is done (update Jacobian)
  */
  void updateJacobian(const FM::Vec& x);

private:
  void init();
  Boost_random rnd;
  SIR_random& genn;
  mutable FM::Vec xp;
  mutable FM::DenseVec n;
  mutable FM::Vec rootq;     // Optimisation of sqrt(q) calculation, automatic on first use
  const Float m_wxSD, m_wySD;
  mutable bool first_init;
};

    //==========================================================================
    //==================== 2D Cartesian observation model=======================
    //==========================================================================

/**
 * Cartesian observation model
 */
class CartesianModel : public JacobianModel,
  public Linrz_correlated_observe_model, public Likelihood_observe_model
{
public:
  /** Size of the state vector */
  static const std::size_t x_size = 4;
  /** Size of the observation vector */
  static const std::size_t z_size = 2;
  /** Predicted observation */
  mutable FM::Vec z_pred;

  /**
  * Contructor
  * @p xSD standard deviation of noise in x
  * @p ySD standard deviation of noise in y
  */
  CartesianModel(Float xSD, Float ySD);

  virtual Float L(const FM::Vec& x) const
  // Definition of likelihood for additive noise model given zz
  {  return li.L(*this, z, h(x));
  }

  virtual void Lz (const FM::Vec& zz)
  // Fix the observation zz about which to evaluate the Likelihood function
  // Zv is also fixed
  {  Likelihood_observe_model::z = zz;
    li.Lz(*this);
  }


  /**
  * Non-linear observation model
  * @param x A-priori estimated state vector x^(k)
  * @return Estimated observations z^(k)
  */
  const FM::Vec& h(const FM::Vec& x) const;

  /**
  * Model update, to be called before observation/correction
  * @param x State vector around which the linearization is done (update Jacobian)
  */
  void updateJacobian(const FM::Vec& x);

  /**
  * Normalize the angular components of the observation model
  * @param z_denorm
  * @param z_from
  */
  void normalise(FM::Vec& z_denorm, const FM::Vec& z_from) const;

private:
  struct Likelihood_correlated
  {
    Likelihood_correlated(std::size_t z_size) :
        zInnov(z_size), Z_inv(z_size,z_size)
    {  zset = false;
    }
    mutable FM::Vec zInnov; // Normailised innovation, temporary for L(x)
    FM::SymMatrix Z_inv; // Inverse Noise Covariance
    Float logdetZ;       // log(det(Z)
    bool zset;
    static Float scaled_vector_square(const FM::Vec& v, const FM::SymMatrix& V);
    Float L(const Correlated_additive_observe_model& model, const FM::Vec& z, const FM::Vec& zp) const;
    // Definition of likelihood for additive noise model given zz
    void Lz(const Correlated_additive_observe_model& model);
  };
  Likelihood_correlated li;
};


    //==========================================================================
    //==================== 2D Polar observation model=======================
    //==========================================================================

/**
 * Polar observation model
 */
class PolarModel : public JacobianModel,
   public Linrz_correlated_observe_model, public Likelihood_observe_model
{
private:
   /** Current sensor pose */
   Float sensor_x;
   Float sensor_y;
   Float sensor_phi;
public:
   /** Size of the state vector */
   static const std::size_t x_size = 4; // state [x vx y vy]
   /** Size of the observation vector */
   static const std::size_t z_size = 2;
   /** Predicted observation */
   mutable FM::Vec z_pred;

  /**
  * Contructor
  * @p bSD standard deviation of bearing noise
  * @p rSD standard deviation of range noise
  */
  PolarModel(Float bSD, Float rSD);

   
   virtual Float L(const FM::Vec& x) const
   // Definition of likelihood for addative noise model given zz
   {  return li.L(*this, z, h(x));
   }
   
   virtual void Lz (const FM::Vec& zz)
   // Fix the observation zz about which to evaluate the Likelihood function
   // Zv is also fixed
   {  Likelihood_observe_model::z = zz;
      li.Lz(*this);
   }
   

   /**
    * Non-linear observation model
    * @param x A-priori estimated state vector x^(k)
    * @return Estimated observations z^(k)
    */
   const FM::Vec& h(const FM::Vec& x) const;

   /**
    * Model update, to be called before observation/correction
    * @param sensor_x sensor x position
    * @param sensor_y sensor y position
    * @param sensor_phi sensor orientation
    */
   void update(const Float& sensor_x, const Float& sensor_y, const Float& sensor_phi);

   /**
    * Model update, to be called before observation/correction
    * @param x State vector around which the linearization is done (update Jacobian)
    */
   void updateJacobian(const FM::Vec& x);

   /**
    * Normalize the angular components of the observation model
    * @param z_denorm 
    * @param z_from 
    */
   void normalise(FM::Vec& z_denorm, const FM::Vec& z_from) const;

private:
   struct Likelihood_correlated
   {
      Likelihood_correlated(std::size_t z_size) :
         zInnov(z_size), Z_inv(z_size,z_size)
      {  zset = false;
      }
      mutable FM::Vec zInnov; // Normailised innovation, temporary for L(x)
      FM::SymMatrix Z_inv;    // Inverse Noise Covariance
      Float logdetZ;          // log(det(Z)
      bool zset;
      static Float scaled_vector_square(const FM::Vec& v, const FM::SymMatrix& V);
      Float L(const Correlated_additive_observe_model& model, const FM::Vec& z, const FM::Vec& zp) const;
      // Definition of likelihood for addative noise model given zz
      void Lz(const Correlated_additive_observe_model& model);
   };
   Likelihood_correlated li;
};

    //==========================================================================
    //============================== 3D CVModel ================================
    //==========================================================================

    class CVModel3D :
    public JacobianModel,
    public Linrz_predict_model,
    public Sampled_predict_model {
    public:
        /** Size of the human state */
        static const std::size_t x_size = 6;
        /** Size of the human state noise */
        static const std::size_t q_size = 3;
        mutable FM::Vec fx;
        double dt;

        /**
         * Contructor
         * @param wxSD Standard deviation of noise_x
         * @param wySD Standard deviation of noise_y
         */
        CVModel3D(Float wxSD, Float wySD, Float wzSD);

        /**
         * Definition of sampler for additive noise model given state x
         *  Generate Gaussian correlated samples
         * Precond: init_GqG, automatic on first use
         */
        virtual const FM::Vec& fw(const FM::Vec& x) const;

        /** initialise predict given a change to q,G
         *  Implementation: Update rootq
         */
        void init_GqG() const;

        /**
         * Non-linear prediction model
         * @param x State vector x(k-1)
         * @return A-priori estimation x^(k)
         */
        const FM::Vec& f(const FM::Vec& x) const;

        /**
         * Model update, to be called before prediction
         * @param dt Time interval since last estimation
         */
        void update(double dt);

        /**
         * Model update, to be called before prediction
         * @param x State vector around which the linearization is done (update Jacobian)
         */
        void updateJacobian(const FM::Vec& x);

    private:
        void init();
        Boost_random rnd;
        SIR_random& genn;
        mutable FM::Vec xp;
        mutable FM::DenseVec n;
        mutable FM::Vec rootq; // Optimisation of sqrt(q) calculation, automatic on first use
        const Float m_wxSD, m_wySD, m_wzSD;
        mutable bool first_init;
    };


    //==========================================================================
    //==================== 3D Cartesian observation model=======================
    //==========================================================================

    class CartesianModel3D : public JacobianModel,
    public Linrz_correlated_observe_model, public Likelihood_observe_model {
    public:
        /** Size of the state vector */
        static const std::size_t x_size = 6;
        /** Size of the observation vector */
        static const std::size_t z_size = 3;
        /** Predicted observation */
        mutable FM::Vec z_pred;

        /**
         * Contructor
         * @p xSD standard deviation of noise in x
         * @p ySD standard deviation of noise in y
         */
        CartesianModel3D(Float xSD, Float ySD, Float zSD);

        virtual Float L(const FM::Vec& x) const
        // Definition of likelihood for additive noise model given zz
        {
            return li.L(*this, z, h(x));
        }

        virtual void Lz(const FM::Vec& zz)
        // Fix the observation zz about which to evaluate the Likelihood function
        // Zv is also fixed
        {
            Likelihood_observe_model::z = zz;
            li.Lz(*this);
        }


        /**
         * Non-linear observation model
         * @param x A-priori estimated state vector x^(k)
         * @return Estimated observations z^(k)
         */
        const FM::Vec& h(const FM::Vec& x) const;

        /**
         * Model update, to be called before observation/correction
         * @param x State vector around which the linearization is done (update Jacobian)
         */
        void updateJacobian(const FM::Vec& x);

        /**
         * Normalize the angular components of the observation model
         * @param z_denorm
         * @param z_from
         */
        void normalise(FM::Vec& z_denorm, const FM::Vec& z_from) const;

    private:

        struct Likelihood_correlated {

            Likelihood_correlated(std::size_t z_size) :
            zInnov(z_size), Z_inv(z_size, z_size) {
                zset = false;
            }
            mutable FM::Vec zInnov; // Normailised innovation, temporary for L(x)
            FM::SymMatrix Z_inv; // Inverse Noise Covariance
            Float logdetZ; // log(det(Z)
            bool zset;
            static Float scaled_vector_square(const FM::Vec& v, const FM::SymMatrix& V);
            Float L(const Correlated_additive_observe_model& model, const FM::Vec& z, const FM::Vec& zp) const;
            // Definition of likelihood for additive noise model given zz
            void Lz(const Correlated_additive_observe_model& model);
        };
        Likelihood_correlated li;
    };


} //namespace


#endif
