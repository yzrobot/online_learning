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
#include "bayes_tracking/ukfilter.h"
#include <boost/numeric/ublas/io.hpp>
#include <float.h>
// #include "utility.h"

#include "bayes_tracking/BayesFilter/matSup.hpp"
#include "bayes_tracking/BayesFilter/bayesFlt.hpp"

using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;


UKFilter::UKFilter(std::size_t x_size) :
        Kalman_state_filter(x_size),
        Unscented_scheme(x_size),
        z_p(Empty)
{
    UKFilter::x_size = x_size;
    UKFilter::XX_size = 2*x_size+1;
    FM::Vec x0(x_size);
    x0.clear();
    FM::SymMatrix P0(x_size, x_size);
    P0.clear();
    init(x0, P0);
}


UKFilter::UKFilter(const FM::Vec& x0, const FM::SymMatrix& P0) :
        Kalman_state_filter(x0.size()),
        Unscented_scheme(x0.size()),
        z_p(Empty)
{
    UKFilter::x_size = x0.size();
    UKFilter::XX_size = 2*x0.size()+1;
    init(x0, P0);
}


UKFilter::~UKFilter()
{
}


void UKFilter::init(const FM::Vec& x0, const FM::SymMatrix& P0)
{
    init_kalman(x0, P0);
}


void UKFilter::update(Additive_predict_model& predict_model,
                      Correlated_additive_observe_model& observe_model,
                      const FM::Vec& z)
{
    predict(predict_model);
    observe(observe_model, z);
}


void UKFilter::predict_observation(Correlated_additive_observe_model& observe_model, FM::Vec& z_pred, FM::SymMatrix& R_pred)
{
    std::size_t z_size = z_pred.size();
    ColMatrix zXX(z_size, 2*x_size+1);
    SymMatrix Xzz(z_size,z_size);
    Matrix Xxz(x_size,z_size);

    z_p.resize(z_size);
    observe_size (z_size);  // Dynamic sizing

    // Create unscented distribution
    kappa = observe_Kappa(x_size);
    Float x_kappa = Float(x_size) + kappa;
    unscented (XX, x, X, x_kappa);

    // Predict points of XX using supplied observation model
    {
        Vec zXXi(z_size), zXX0(z_size);
        zXX0 = static_cast<Correlated_additive_observe_model&>(observe_model).h( column(XX,0) );
        column(zXX,0) = zXX0;
        for (std::size_t i = 1; i < XX.size2(); ++i) {
            zXXi = static_cast<Correlated_additive_observe_model&>(observe_model).h( column(XX,i) );
            // Normalise relative to zXX0
            observe_model.normalise (zXXi, zXX0);
            column(zXX,i) = zXXi;
        }
    }

    // Mean of predicted distribution: z_p
    noalias(z_p) = column(zXX,0) * kappa;
    for (std::size_t i = 1; i < zXX.size2(); ++i) {
        noalias(z_p) += column(zXX,i) / Float(2); // ISSUE uBlas may not be able to promote integer 2
    }
    z_pred = z_p /= x_kappa;

    // Covariance of observation predict: Xzz
    // Subtract mean from each point in zXX
    for (std::size_t i = 0; i < XX_size; ++i) {
        column(zXX,i).minus_assign (z_p);
    }
    // Center point, premult here by 2 for efficency
    {
        ColMatrix::Column zXX0 = column(zXX,0);
        noalias(Xzz) = FM::outer_prod(zXX0, zXX0);
        Xzz *= 2*kappa;
    }
    // Remaining unscented points
    for (std::size_t i = 1; i < zXX.size2(); ++i) {
        ColMatrix::Column zXXi = column(zXX,i);
        noalias(Xzz) += FM::outer_prod(zXXi, zXXi);
    }
    Xzz /= 2*x_kappa;


    /** Fix for non-positive semidefinite covariance **/
    {
        ColMatrix::Column zXX0 = column(zXX,0);      // mean zp already subtracted above
        noalias(Xzz) += FM::outer_prod(zXX0, zXX0);  // modified covariance about mean
    }


    R_pred = Xzz;

//    // Correlation of state with observation: Xxz
//    // Center point, premult here by 2 for efficency
//    {
//       noalias(Xxz) = FM::outer_prod(column(XX,0) - x, column(zXX,0));
//       Xxz *= 2*kappa;
//    }
//    // Remaining unscented points
//    for (std::size_t i = 1; i < zXX.size2(); ++i) {
//       noalias(Xxz) += FM::outer_prod(column(XX,i) - x, column(zXX,i));
//    }
//    Xxz /= 2* (Float(x_size) + kappa);
}


Bayes_base::Float UKFilter::observeInnovation(Correlated_additive_observe_model& h, const Vec& si, const SymMatrix& Si)
/*
 * Observation fusion
 *  Pre : x,X
 *  Post: x,X is PSD
 */
{
    std::size_t z_size = si.size();
    ColMatrix zXX (z_size, 2*x_size+1);
    Vec zp(z_size);
    SymMatrix Xzz(z_size,z_size);
    Matrix Xxz(x_size,z_size);
    Matrix W(x_size,z_size);

    observe_size (si.size());   // Dynamic sizing

    // Create unscented distribution
    kappa = observe_Kappa(x_size);
    Float x_kappa = Float(x_size) + kappa;
    unscented (XX, x, X, x_kappa);

    // Predict points of XX using supplied observation model
    {
        Vec zXXi(z_size), zXX0(z_size);
        zXX0 = h.h( column(XX,0) );
        column(zXX,0) = zXX0;
        for (std::size_t i = 1; i < XX.size2(); ++i) {
            zXXi = h.h( column(XX,i) );
            // Normalise relative to zXX0
            h.normalise (zXXi, zXX0);
            column(zXX,i) = zXXi;
        }
    }

    // Mean of predicted distribution: zp
    noalias(zp) = column(zXX,0) * kappa;
    for (std::size_t i = 1; i < zXX.size2(); ++i) {
        noalias(zp) += column(zXX,i) / Float(2); // ISSUE uBlas may not be able to promote integer 2
    }
    zp /= x_kappa;

    // Covariance of observation predict: Xzz
    // Subtract mean from each point in zXX
    for (std::size_t i = 0; i < XX_size; ++i) {
        column(zXX,i).minus_assign (zp);
    }
    // Center point, premult here by 2 for efficency
    {
        ColMatrix::Column zXX0 = column(zXX,0);
        noalias(Xzz) = FM::outer_prod(zXX0, zXX0);
        Xzz *= 2*kappa;
    }
    // Remaining unscented points
    for (std::size_t i = 1; i < zXX.size2(); ++i) {
        ColMatrix::Column zXXi = column(zXX,i);
        noalias(Xzz) += FM::outer_prod(zXXi, zXXi);
    }
    Xzz /= 2*x_kappa;


    /** Fix for non-positive semidefinite covariance **/
    {
        ColMatrix::Column zXX0 = column(zXX,0);      // mean zp already subtracted above
        noalias(Xzz) += FM::outer_prod(zXX0, zXX0);  // modified covariance about mean
    }


    // Correlation of state with observation: Xxz
    // Center point, premult here by 2 for efficency
    {
        noalias(Xxz) = FM::outer_prod(column(XX,0) - x, column(zXX,0));
        Xxz *= 2*kappa;
    }
    // Remaining unscented points
    for (std::size_t i = 1; i < zXX.size2(); ++i) {
        noalias(Xxz) += FM::outer_prod(column(XX,i) - x, column(zXX,i));
    }
    Xxz /= 2* (Float(x_size) + kappa);

    // Innovation covariance
    S = Xzz;
    noalias(S) += h.Z;
    // Inverse innovation covariance
    Float rcond = UdUinversePD (SI, S);
    rclimit.check_PD(rcond, "S not PD in observeInnovation");
    // Kalman gain
    noalias(W) = prod(Xxz,SI);

    // Store innovation
    noalias(s) = si;

    // Filter update
    noalias(x) += prod(W,s);
    RowMatrix WStemp(W.size1(), S.size2());
    // update state cov with modified innovation cov
    noalias(X) -= prod_SPD(W, Si, WStemp);

    return rcond;
}



double UKFilter::logLikelihood() {
    SymMatrix Si(S.size1(), S.size2());
    Float detS;
    Float rcond = UdUinversePD(Si, detS, S);  // Si = inv(S)
    Numerical_rcond rclimit;
    rclimit.check_PD(rcond, "S not PD in UKFilter::logLikelihood");
    // exp(-0.5 * (s' * Si * s)) / sqrt(2pi^ns * |S|)
    return -0.5*(inner_prod(trans(s),prod(Si, s))) - 0.5*((double)s.size()*log(2*M_PI)+log(detS));
}


void UKFilter::unscented(FM::ColMatrix& XX, const FM::Vec& x, const FM::SymMatrix& X, Float scale) {
    UTriMatrix Sigma(x_size, x_size);

    // Get a upper Cholesky factoriation
    Float rcond = UCfactor(Sigma, X);
    rclimit.check_PSD(rcond, "X not PSD in UKFilter::unscented(...)");
    Sigma *= std::sqrt(scale);

    // Generate XX with the same sample Mean and Covar as before
    column(XX,0) = x;

    for (std::size_t c = 0; c < x_size; ++c) {
        UTriMatrix::Column SigmaCol = column(Sigma,c);
        noalias(column(XX,c+1)) = x  + SigmaCol;
        noalias(column(XX,x_size+c+1)) = x - SigmaCol;
    }
}
