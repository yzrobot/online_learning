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

#include "bayes_tracking/models.h"
#include "bayes_tracking/BayesFilter/matSup.hpp"
#include "bayes_tracking/angle.h"
#include <boost/numeric/ublas/io.hpp>
#include <float.h>


using namespace Bayesian_filter;
using namespace Models;


//*************************************************************************
//                      CV PREDICTION MODEL
//*************************************************************************

CVModel::CVModel(Float wxSD, Float wySD) :
   Linrz_predict_model(x_size, q_size),
   Sampled_predict_model(),
   fx(x_size),
   genn(rnd),
   xp(x_size),
   n(q_size), rootq(q_size),
   m_wxSD(wxSD), m_wySD(wySD)
{
   first_init = true;
   init();
}


void CVModel::init()
{
  Fx.clear();
  // x
  Fx(0,0) = 1.;
  Fx(0,1) = dt;
  Fx(0,2) = 0.;
  Fx(0,3) = 0.;
  // dx
  Fx(1,0) = 0.;
  Fx(1,1) = 1.;
  Fx(1,2) = 0.;
  Fx(1,3) = 0.;
  // y
  Fx(2,0) = 0.;
  Fx(2,1) = 0.;
  Fx(2,2) = 1.;
  Fx(2,3) = dt;
  // dy
  Fx(3,0) = 0.;
  Fx(3,1) = 0.;
  Fx(3,2) = 0.;
  Fx(3,3) = 1.;
  // noise
  q[0] = sqr(m_wxSD); // cov(w_x)
  q[1] = sqr(m_wySD); // cov(w_y)
  G.clear();
  G(0,0) = 0.5*sqr(dt);
  G(1,0) = dt;
  G(2,1) = 0.5*sqr(dt);
  G(3,1) = dt;
}


const FM::Vec& CVModel::f(const FM::Vec& x) const
{  // human model
   fx[0] = x[0] + x[1] * dt;  // x
   fx[1] = x[1];              // dx
   fx[2] = x[2] + x[3] * dt;  // y
   fx[3] = x[3];              // dy
   return fx;
}


void CVModel::update(double dt)
{
  // time interval
  this->dt = dt;

  //     | 0.5dt^2     0    |
  // G = |    dt       0    |
  //     |    0     0.5dt^2 |
  //     |    0        dt   |
  G(0,0) = 0.5*sqr(dt);
  G(1,0) = dt;
  G(2,1) = 0.5*sqr(dt);
  G(3,1) = dt;
}


void CVModel::updateJacobian(const FM::Vec& x) {
   //      | 1  dt 0  0 |
   // Fx = | 0  1  0  0 |
   //      | 0  0  1  dt|
   //      | 0  0  0  1 |
  Fx(0,1) = dt;
  Fx(2,3) = dt;
}


const Vec& CVModel::fw(const FM::Vec& x) const
/*
   * Definition of sampler for additive noise model given state x
   *  Generate Gaussian correlated samples
   * Precond: init_GqG, automatic on first use
   */
{
   if (first_init)
      init_GqG();
                  // Predict state using supplied functional predict model
   xp = f(x);
                  // Additive random noise
   CVModel::genn.normal(n);            // independant zero mean normal
                        // multiply elements by std dev
   for (FM::DenseVec::iterator ni = n.begin(); ni != n.end(); ++ni) {
      *ni *= rootq[ni.index()];
   }
   FM::noalias(xp) += FM::prod(this->G,n);         // add correlated noise
   return xp;
}


void CVModel::init_GqG() const
/* initialise predict given a change to q,G
   *  Implementation: Update rootq
   */
{
   first_init = false;
   for (FM::Vec::const_iterator qi = this->q.begin(); qi != this->q.end(); ++qi) {
      if (*qi < 0)
         error (Numeric_exception("Negative q in init_GqG"));
      rootq[qi.index()] = std::sqrt(*qi);
   }
}



//*************************************************************************
//                  2D CARTESIAN OBSERVATION MODEL
//*************************************************************************

CartesianModel::CartesianModel(Float xSD, Float ySD) :
   Linrz_correlated_observe_model(x_size, z_size),
   Likelihood_observe_model(z_size),
   z_pred(z_size),
   li(z_size)
{
  // Hx = | 1  0  0  0 |
  //      | 0  0  1  0 |
  Hx.clear();
  Hx(0,0) = 1.;
  Hx(1,2) = 1.;
  // noise
  Z.clear();
  Z(0,0) = sqr(xSD);
  Z(1,1) = sqr(ySD);
}

Bayes_base::Float
 CartesianModel::Likelihood_correlated::L(const Correlated_additive_observe_model& model, const FM::Vec& z, const FM::Vec& zp) const
/*
 * Definition of likelihood given an additive Gaussian observation model:
 *  p(z|x) = exp(-0.5*(z-h(x))'*inv(Z)*(z-h(x))) / sqrt(2pi^nz*det(Z));
 *  L(x) the the Likelihood L(x) doesn't depend on / sqrt(2pi^nz) for constant z size
 * Precond: Observation Information: z,Z_inv,detZterm
 */
{
   if (!zset)
      Bayes_base::error (Logic_exception ("BGSubModel used without Lz set"));
               // Normalised innovation
   zInnov = z;
   model.normalise (zInnov, zp);
   FM::noalias(zInnov) -= zp;

   Float logL = scaled_vector_square(zInnov, Z_inv);
   using namespace std;
   return exp(Float(-0.5)*(logL + z.size()*log(2*M_PI) + logdetZ));   // normalized likelihood
}


void CartesianModel::Likelihood_correlated::Lz (const Correlated_additive_observe_model& model)
/* Set the observation zz and Z about which to evaluate the Likelihood function
 * Postcond: Observation Information: z,Z_inv,detZterm
 */
{
   zset = true;
                  // Compute inverse of Z and its reciprocal condition number
   Float detZ;
   Float rcond = FM::UdUinversePD (Z_inv, detZ, model.Z);
   model.rclimit.check_PD(rcond, "Z not PD in observe");
                  // detZ > 0 as Z PD
   using namespace std;
   logdetZ = log(detZ);
}


Bayes_base::Float
 CartesianModel::Likelihood_correlated::scaled_vector_square(const FM::Vec& v, const FM::SymMatrix& V)
/*
 * Compute covariance scaled square inner product of a Vector: v'*V*v
 */
{
   return FM::inner_prod(v, FM::prod(V,v));
}


const FM::Vec& CartesianModel::h(const FM::Vec& x) const
{
  z_pred[0] = x[0];
  z_pred[1] = x[2];
  return z_pred;
};
   

void CartesianModel::updateJacobian(const FM::Vec& x) {
  // nothing to do
}


void CartesianModel::normalise(FM::Vec& z_denorm, const FM::Vec& z_from) const {
}


//*************************************************************************
//                  2D POLAR OBSERVATION MODEL
//*************************************************************************

PolarModel::PolarModel(Float bSD, Float rSD) :
   Linrz_correlated_observe_model(x_size, z_size),
   Likelihood_observe_model(z_size),
   z_pred(z_size),
   li(z_size)
{
   Hx.clear();
   // noise
   Z.clear();
   Z(0,0) = sqr(bSD);
   Z(1,1) = sqr(rSD);
}

Bayes_base::Float
 PolarModel::Likelihood_correlated::L(const Correlated_additive_observe_model& model, const FM::Vec& z, const FM::Vec& zp) const
/*
 * Definition of likelihood given an additive Gaussian observation model:
 *  p(z|x) = exp(-0.5*(z-h(x))'*inv(Z)*(z-h(x))) / sqrt(2pi^nz*det(Z));
 *  L(x) the the Likelihood L(x) doesn't depend on / sqrt(2pi^nz) for constant z size
 * Precond: Observation Information: z,Z_inv,detZterm
 */
{
   if (!zset)
      Bayes_base::error (Logic_exception ("PolarModel used without Lz set"));
               // Normalised innovation
   zInnov = z;
   model.normalise (zInnov, zp);
   FM::noalias(zInnov) -= zp;

   Float logL = scaled_vector_square(zInnov, Z_inv);
   using namespace std;
   return exp(Float(-0.5)*(logL + z.size()*log(2*M_PI) + logdetZ));   // normalized likelihood
}


void PolarModel::Likelihood_correlated::Lz (const Correlated_additive_observe_model& model)
/* Set the observation zz and Z about which to evaluate the Likelihood function
 * Postcond: Observation Information: z,Z_inv,detZterm
 */
{
   zset = true;
                  // Compute inverse of Z and its reciprocal condition number
   Float detZ;
   Float rcond = FM::UdUinversePD (Z_inv, detZ, model.Z);
   model.rclimit.check_PD(rcond, "Z not PD in observe");
                  // detZ > 0 as Z PD
   using namespace std;
   logdetZ = log(detZ);
}


Bayes_base::Float
 PolarModel::Likelihood_correlated::scaled_vector_square(const FM::Vec& v, const FM::SymMatrix& V)
/*
 * Compute covariance scaled square inner product of a Vector: v'*V*v
 */
{
   return FM::inner_prod(v, FM::prod(V,v));
}


const FM::Vec& PolarModel::h(const FM::Vec& x) const
{
   z_pred[0] = atan2(x[2] - sensor_y, x[0] - sensor_x) - sensor_phi;   // bearing
   z_pred[1] = sqrt(sqr(x[0] - sensor_x) + sqr(x[2] - sensor_y));      // distance
   
   return z_pred;
};
   

void PolarModel::update(const Float& sensor_x, const Float& sensor_y, const Float& sensor_phi)
{
   this->sensor_x = sensor_x;
   this->sensor_y = sensor_y;
   this->sensor_phi = sensor_phi;
}


void PolarModel::updateJacobian(const FM::Vec& x) {
   // update Jacobian
   double ds2 = sqr(x[0] - sensor_x) + sqr(x[2] - sensor_y);
   
   // Hx = | *  0  *  0 |
   //      | *  0  *  0 |

   Hx(0,0) = - (x[2] - sensor_y) / (ds2 + DBL_EPSILON);
   Hx(0,2) = (x[0] - sensor_x) / (ds2 + DBL_EPSILON);

   Hx(1,0) = (x[0] - sensor_x) / (sqrt(ds2) + DBL_EPSILON);
   Hx(1,2) = (x[2] - sensor_y) / (sqrt(ds2) + DBL_EPSILON);
}


void PolarModel::normalise(FM::Vec& z_denorm, const FM::Vec& z_from) const
{
   z_denorm[0] = angleArith::angle<Float>(z_denorm[0]).from (z_from[0]);
}


//*************************************************************************
//                      CV 3D PREDICTION MODEL
//*************************************************************************

   CVModel3D::CVModel3D(Float wxSD, Float wySD, Float wzSD) :
   Linrz_predict_model(x_size, q_size),
   Sampled_predict_model(),
   fx(x_size),
   genn(rnd),
   xp(x_size),
   n(q_size), rootq(q_size),
    m_wxSD(wxSD), m_wySD(wySD),m_wzSD(wzSD)
{
   first_init = true;
   init();
}


void CVModel3D::init()
{
  Fx.clear();
  // x
  Fx(0,0) = 1.;
  Fx(0,1) = dt;
  Fx(0,2) = 0.;
  Fx(0,3) = 0.;
  Fx(0,4) = 0.;
  Fx(0,5) = 0.;
  // dx
  Fx(1,0) = 0.;
  Fx(1,1) = 1.;
  Fx(1,2) = 0.;
  Fx(1,3) = 0.;
  Fx(1,4) = 0.;
  Fx(1,5) = 0.;
  // y
  Fx(2,0) = 0.;
  Fx(2,1) = 0.;
  Fx(2,2) = 1.;
  Fx(2,3) = dt;
  Fx(2,4) = 0.;
  Fx(2,5) = 0.;
  // dy
  Fx(3,0) = 0.;
  Fx(3,1) = 0.;
  Fx(3,2) = 0.;
  Fx(3,3) = 1.;
  Fx(3,4) = 0.;
  Fx(3,5) = 0.;
  // z
  Fx(4,0) = 0.;
  Fx(4,1) = 0.;
  Fx(4,2) = 0.;
  Fx(4,3) = 0.;
  Fx(4,4) = 1.;
  Fx(4,5) = dt;
  // dz
  Fx(5,0) = 0.;
  Fx(5,1) = 0.;
  Fx(5,2) = 0.;
  Fx(5,3) = 0.;
  Fx(5,4) = 0.;
  Fx(5,5) = 1.;

  // noise
  q[0] = sqr(m_wxSD); // cov(w_x)
  q[1] = sqr(m_wySD); // cov(w_y)
  q[2] = sqr(m_wzSD); // cov(w_z)
  G.clear();
  G(0,0) = 0.5*sqr(dt);
  G(1,0) = dt;
  G(2,1) = 0.5*sqr(dt);
  G(3,1) = dt;
  G(4,2) = 0.5*sqr(dt);
  G(5,2) = dt;
}

const FM::Vec& CVModel3D::f(const FM::Vec& x) const
{  // human model
   fx[0] = x[0] + x[1] * dt;  // x
   fx[1] = x[1];              // dx
   fx[2] = x[2] + x[3] * dt;  // y
   fx[3] = x[3];              // dy
   fx[4] = x[4] + x[5] * dt;  // z
   fx[5] = x[5];              // dz
   return fx;
}


void CVModel3D::update(double dt)
{
  // time interval
  this->dt = dt;

  //     | 0.5dt^2     0    |
  // G = |    dt       0    |
  //     |    0     0.5dt^2 |
  //     |    0        dt   |
  G(0,0) = 0.5*sqr(dt);
  G(1,0) = dt;
  G(2,1) = 0.5*sqr(dt);
  G(3,1) = dt;
  G(4,2) = 0.5*sqr(dt);
  G(5,2) = dt;
}


void CVModel3D::updateJacobian(const FM::Vec& x) {
   //      | 1  dt 0  0 |
   // Fx = | 0  1  0  0 |
   //      | 0  0  1  dt|
   //      | 0  0  0  1 |
  Fx(0,1) = dt;
  Fx(2,3) = dt;
  Fx(4,5) = dt;
}


const Vec& CVModel3D::fw(const FM::Vec& x) const
/*
   * Definition of sampler for additive noise model given state x
   *  Generate Gaussian correlated samples
   * Precond: init_GqG, automatic on first use
   */
{
   if (first_init)
      init_GqG();
                  // Predict state using supplied functional predict model
   xp = f(x);
                  // Additive random noise
   CVModel3D::genn.normal(n);            // independant zero mean normal
                        // multiply elements by std dev
   for (FM::DenseVec::iterator ni = n.begin(); ni != n.end(); ++ni) {
      *ni *= rootq[ni.index()];
   }
   FM::noalias(xp) += FM::prod(this->G,n);         // add correlated noise
   return xp;
}


void CVModel3D::init_GqG() const
/* initialise predict given a change to q,G
   *  Implementation: Update rootq
   */
{
   first_init = false;
   for (FM::Vec::const_iterator qi = this->q.begin(); qi != this->q.end(); ++qi) {
      if (*qi < 0)
         error (Numeric_exception("Negative q in init_GqG"));
      rootq[qi.index()] = std::sqrt(*qi);
   }
}


//*************************************************************************
//                 3D  CARTESIAN SUBTRACTION OBSERVATION MODEL
//*************************************************************************

CartesianModel3D::CartesianModel3D(Float xSD, Float ySD, Float zSD) :
   Linrz_correlated_observe_model(x_size, z_size),
   Likelihood_observe_model(z_size),
   z_pred(z_size),
   li(z_size)
{
  // Hx = | 1  0  0  0 |
  //      | 0  0  1  0 |
  Hx.clear();
  Hx(0,0) = 1.;
  Hx(1,2) = 1.;
  Hx(2,4) = 1.;
  // noise
  Z.clear();
  Z(0,0) = sqr(xSD);
  Z(1,1) = sqr(ySD);
  Z(2,2) = sqr(zSD);
}

Bayes_base::Float
 CartesianModel3D::Likelihood_correlated::L(const Correlated_additive_observe_model& model, const FM::Vec& z, const FM::Vec& zp) const
/*
 * Definition of likelihood given an additive Gaussian observation model:
 *  p(z|x) = exp(-0.5*(z-h(x))'*inv(Z)*(z-h(x))) / sqrt(2pi^nz*det(Z));
 *  L(x) the the Likelihood L(x) doesn't depend on / sqrt(2pi^nz) for constant z size
 * Precond: Observation Information: z,Z_inv,detZterm
 */
{
   if (!zset)
      Bayes_base::error (Logic_exception ("BGSubModel used without Lz set"));
               // Normalised innovation
   zInnov = z;
   model.normalise (zInnov, zp);
   FM::noalias(zInnov) -= zp;

   Float logL = scaled_vector_square(zInnov, Z_inv);
   using namespace std;
   return exp(Float(-0.5)*(logL + z.size()*log(2*M_PI) + logdetZ));   // normalized likelihood
}


void CartesianModel3D::Likelihood_correlated::Lz (const Correlated_additive_observe_model& model)
/* Set the observation zz and Z about which to evaluate the Likelihood function
 * Postcond: Observation Information: z,Z_inv,detZterm
 */
{
   zset = true;
                  // Compute inverse of Z and its reciprocal condition number
   Float detZ;
   Float rcond = FM::UdUinversePD (Z_inv, detZ, model.Z);
   model.rclimit.check_PD(rcond, "Z not PD in observe");
                  // detZ > 0 as Z PD
   using namespace std;
   logdetZ = log(detZ);
}


Bayes_base::Float
 CartesianModel3D::Likelihood_correlated::scaled_vector_square(const FM::Vec& v, const FM::SymMatrix& V)
/*
 * Compute covariance scaled square inner product of a Vector: v'*V*v
 */
{
   return FM::inner_prod(v, FM::prod(V,v));
}


const FM::Vec& CartesianModel3D::h(const FM::Vec& x) const
{
  z_pred[0] = x[0];
  z_pred[1] = x[2];
  z_pred[2] = x[4];
  return z_pred;
};
   

void CartesianModel3D::updateJacobian(const FM::Vec& x) {
  // nothing to do
}


void CartesianModel3D::normalise(FM::Vec& z_denorm, const FM::Vec& z_from) const {
}
