//
// C++ Interface: jacobianmodel
//
// Description: 
//
//
// Author: Nicola Bellotto <nbellotto@lincoln.ac.uk>, (C) 2011
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef JACOBIANMODEL_H
#define JACOBIANMODEL_H

#include "bayes_tracking/BayesFilter/bayesFlt.hpp"

using namespace Bayesian_filter_matrix;

class JacobianModel {
public:
  /** Contructor  */
  virtual ~JacobianModel(){};

  /**
   * Update Jacobian matrix of the model
   * @p x state argument
   */
  virtual void updateJacobian(const Vec& x) = 0;
};


#endif  // JACOBIANMODEL_H
