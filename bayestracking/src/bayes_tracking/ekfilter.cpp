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
#include "bayes_tracking/ekfilter.h"
#include <boost/numeric/ublas/io.hpp>
#include <float.h>
// #include "utility.h"
#include "bayes_tracking/jacobianmodel.h"

#include "bayes_tracking/BayesFilter/matSup.hpp"
#include "bayes_tracking/BayesFilter/bayesFlt.hpp"

using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;


EKFilter::EKFilter(std::size_t x_size) :
        Kalman_state_filter(x_size),
        Covariance_scheme(x_size),
        s(Empty), z_p(Empty)
{
    EKFilter::x_size = x_size;
    FM::Vec x0(x_size);
    x0.clear();
    FM::SymMatrix P0(x_size, x_size);
    P0.clear();
    init(x0, P0);
}


EKFilter::EKFilter(const FM::Vec& x0, const FM::SymMatrix& P0) :
        Kalman_state_filter(x0.size()),
        Covariance_scheme(x0.size()),
        s(Empty), z_p(Empty)
{
    EKFilter::x_size = x0.size();
    init(x0, P0);
}


EKFilter::~EKFilter()
{
}


void EKFilter::init(const FM::Vec& x0, const FM::SymMatrix& P0)
{
    init_kalman(x0, P0);
}


Bayes_base::Float EKFilter::predict (Linrz_predict_model& f) {
    dynamic_cast<JacobianModel&>(f).updateJacobian(x); // update model linearization
    return Covariance_scheme::predict(f);
}


Bayes_base::Float EKFilter::predict (Gaussian_predict_model& f) {
    dynamic_cast<JacobianModel&>(f).updateJacobian(x); // update model linearization
    return Covariance_scheme::predict(f);
}


void EKFilter::update(Linrz_predict_model& predict_model,
                      Linrz_correlated_observe_model& observe_model,
                      const FM::Vec& z)
{
    dynamic_cast<JacobianModel&>(predict_model).updateJacobian(x); // update model linearization
    predict(predict_model);

    dynamic_cast<JacobianModel&>(observe_model).updateJacobian(x); // update model linearization
    observe(observe_model, z);
}


void EKFilter::predict_observation(Linrz_correlated_observe_model& observe_model, FM::Vec& z_pred, FM::SymMatrix& R_pred)
{
    dynamic_cast<JacobianModel&>(observe_model).updateJacobian(x); // update model linearization
    z_pred = observe_model.h(x);  // predicted observation
    // covariance of predicted observation
    Bayesian_filter_matrix::Matrix dum(prod(X, trans(observe_model.Hx)));
    noalias(R_pred) = prod(observe_model.Hx, dum);
}


Bayes_base::Float EKFilter::observeInnovation(Linrz_correlated_observe_model& h, const Bayesian_filter_matrix::Vec& si, const Bayesian_filter_matrix::SymMatrix& Si)
/*
 * Extended linrz correlated observe, compute innovation for observe_innovation
 */
{
    dynamic_cast<JacobianModel&>(h).updateJacobian(x); // update model linearization

    if (last_z_size != si.size()) {
        s.resize(si.size());
    }
    FM::noalias(s) = si;

    // Size consistency, z to model
    if (s.size() != h.Z.size1())
        error (Logic_exception("observation and model size inconsistent in observeInnovation"));
    observe_size (s.size());// Dynamic sizing

    // Innovation covariance
    Bayesian_filter_matrix::Matrix temp_XZ (prod(X, trans(h.Hx)));
    noalias(S) = prod(h.Hx, temp_XZ) + h.Z;

    // Inverse innovation covariance
    Float rcond = UdUinversePD (SI, S);
    rclimit.check_PD(rcond, "S not PD in observeInnovation");

    // Kalman gain, X*Hx'*SI
    noalias(W) = prod(temp_XZ, SI);

    // State update
    noalias(x) += prod(W, s);
    // update state cov with modified innovation cov
    noalias(X) -= prod_SPD(W, Si, temp_XZ);   // temp_XZ used just as temporary storage matrix

    return rcond;
}


Bayes_base::Float
EKFilter::observe (Linrz_correlated_observe_model& h, const FM::Vec& z)
/*
 * Extended linrz correlated observe, compute innovation for observe_innovation
 */
{
    dynamic_cast<JacobianModel&>(h).updateJacobian(x); // update model linearization

    Covariance_scheme::update ();
    const FM::Vec& zp = h.h(x);      // Observation model, zp is predicted observation

    if (last_z_size != z.size()) {
        s.resize(z.size());
    }
    s = z;
    h.normalise(s, zp);
    FM::noalias(s) -= zp;

    return observe_innovation (h, s);
}


Bayes_base::Float
EKFilter::observe (Linrz_uncorrelated_observe_model& h, const FM::Vec& z)
/*
 * Extended kalman uncorrelated observe, compute innovation for observe_innovation
 */
{
    dynamic_cast<JacobianModel&>(h).updateJacobian(x); // update model linearization

    Covariance_scheme::update ();
    const FM::Vec& zp = h.h(x);      // Observation model, zp is predicted observation

    if (last_z_size != z.size()) {
        s.resize(z.size());
    }
    s = z;
    h.normalise(s, zp);
    FM::noalias(s) -= zp;
    return observe_innovation (h, s);
}


double EKFilter::logLikelihood() {
    SymMatrix Si(S.size1(), S.size2());
    Float detS;
    Float rcond = UdUinversePD(Si, detS, S);  // Si = inv(S)
    Numerical_rcond rclimit;
    rclimit.check_PD(rcond, "S not PD in EKFilter::logLikelihood");
    // exp(-0.5 * (s' * Si * s)) / sqrt(2pi^ns * |S|)
    return -0.5*(inner_prod(trans(s),prod(Si, s))) - 0.5*((double)s.size()*log(2*M_PI)+log(detS));
}
