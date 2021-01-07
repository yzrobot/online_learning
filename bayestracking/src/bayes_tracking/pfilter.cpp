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
#include "bayes_tracking/pfilter.h"

PFilter::PFilter(std::size_t x_size, std::size_t s_size, SIR_random& random_helper) :
        Sample_state_filter (x_size, s_size),
        Kalman_state_filter(x_size),
        SIR_kalman_scheme(x_size, s_size, random_helper),
        w(wir)
{
    PFilter::x_size = x_size;
    FM::Vec x0(x_size);
    x0.clear();
    FM::SymMatrix P0(x_size, x_size);
    P0.clear();
    init(x0, P0);
}


PFilter::PFilter(std::size_t x_size, std::size_t s_size) :
        Sample_state_filter (x_size, s_size),
        Kalman_state_filter(x_size),
        SIR_kalman_scheme(x_size, s_size, rnd),
        w(wir)
{
    PFilter::x_size = x_size;
    FM::Vec x0(x_size);
    x0.clear();
    FM::SymMatrix P0(x_size, x_size);
    P0.clear();
    init(x0, P0);
}


PFilter::PFilter(const FM::Vec& x0,
                 const FM::SymMatrix& P0,
                 std::size_t s_size,
                 SIR_random& random_helper) :
        Sample_state_filter (x0.size(), s_size),
        Kalman_state_filter(x0.size()),
        SIR_kalman_scheme(x0.size(), s_size, random_helper),
        w(wir)
{
    PFilter::x_size = x0.size();
    init(x0, P0);
}


PFilter::PFilter(const FM::Vec& x0,
                 const FM::SymMatrix& P0,
                 std::size_t s_size) :
        Sample_state_filter (x0.size(), s_size),
        Kalman_state_filter(x0.size()),
        SIR_kalman_scheme(x0.size(), s_size, rnd),
        w(wir)
{
    PFilter::x_size = x0.size();
    init(x0, P0);
}


PFilter::~PFilter()
{
}


void PFilter::init(const FM::Vec& x0, const FM::SymMatrix& P0)
{
    SIR_kalman_scheme::init_kalman(x0, P0);
    m_likelihood = 0.;
}


void PFilter::predict(Sampled_predict_model& predict_model)
{
    SIR_kalman_scheme::predict(predict_model);
    update_statistics();
}


void PFilter::predict_observation(Correlated_additive_observe_model& observe_model, FM::Vec& z_pred, FM::SymMatrix& R_pred)
{
    z_pred.clear();   // mean
    const std::size_t nSamples = S.size2();
    for (std::size_t i = 0; i != nSamples; ++i) {
        FM::ColMatrix::Column Si(S,i);
        z_pred.plus_assign (observe_model.h(Si));
    }
    z_pred /= Float(S.size2());

    R_pred.clear();   // Covariance
    for (std::size_t i = 0; i != nSamples; ++i) {
        FM::ColMatrix::Column Si(S,i);
        R_pred.plus_assign (FM::outer_prod(observe_model.h(Si)-z_pred, observe_model.h(Si)-z_pred));
    }
    R_pred /= Float(nSamples);
}


void PFilter::observe(Likelihood_observe_model& observe_model, const FM::Vec& z)
{
    SIR_kalman_scheme::observe(observe_model, z);
    // keep note of the weight sum (non-normalized likelihood)
    const std::size_t nSamples = S.size2();
    m_likelihood = 0;
    for (std::size_t i = 0; i != nSamples; ++i) {
        m_likelihood += wir[i];
    }
    // resample
    SIR_kalman_scheme::update_resample(/*Systematic_resampler()*/);
}


double PFilter::logLikelihood() {
    return log(m_likelihood) - log(S.size2());   // normalized likelihood (S are samples, not covariance)
}
