//
// C++ Interface: multitracker
//
// Description: 
//
// Author: Nicola Bellotto <nbellotto@lincoln.ac.uk>, (C) 2011
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef MULTITRACKER_H
#define MULTITRACKER_H

#include "bayes_tracking/BayesFilter/bayesFlt.hpp"
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <boost/numeric/ublas/io.hpp>
#include "bayes_tracking/associationmatrix.h"
#include "bayes_tracking/jpda.h"
#include <float.h>

#define OL // online learning (@yz17iros)

using namespace Bayesian_filter;

namespace MTRK {
  
  struct observation_t {
    FM::Vec vec;
    double time;
    string flag;
    string name;
    // constructors
  observation_t() : vec(Empty), time(0.) {}
  observation_t(FM::Vec v) : vec(v) {}
  observation_t(FM::Vec v, double t) : vec(v), time(t) {}
  observation_t(FM::Vec v, double t, string f) : vec(v), time(t), flag(f) {}
  observation_t(FM::Vec v, double t, string f, string n) : vec(v), time(t), flag(f), name(n) {}
  };
  
  typedef std::vector<observation_t> sequence_t;
  typedef enum {NN, /*JPDA,*/ NNJPDA} association_t;
  typedef enum {CARTESIAN, POLAR} observ_model_t;
  
  // to be defined by user
  template<class FilterType>
    extern bool isLost(const FilterType* filter, double stdLimit = 1.0);
  
  template<class FilterType>
    extern bool initialize(FilterType* &filter, sequence_t& obsvSeq, observ_model_t om_flag);
  
  /**
     @author Nicola Bellotto <nick@robots.ox.ac.uk>
  */
  template<class FilterType, int xSize>
    class MultiTracker {
    
  public:
    typedef struct {
      unsigned long id;
      FilterType* filter;
#ifdef OL
      string pose_id;
      string detector_name;
#endif
    } filter_t;
    
  private:
    std::vector<filter_t> m_filters;
    int m_filterNum;
    sequence_t m_observations;            // observations
    std::vector<size_t> m_unmatched;      // unmatched observations
    std::map<int, int> m_assignments;     // assignment < observation, target >
    std::vector<sequence_t> m_sequences;  // vector of unmatched observation sequences
    
  public:
    /**
     * Constructor
     */
    MultiTracker()
      {
	m_filterNum = 0;
      }
    
    /**
     * Destructor
     */
    ~MultiTracker()
      {
	typename std::vector<filter_t>::iterator fi, fiEnd = m_filters.end();
	for (fi = m_filters.begin(); fi != fiEnd; fi++)
	  delete fi->filter;
      }
    
    /**
     * Add a new observation
     * @param z Observation vector
     * @param time Timestamp
     * @param flag Additional flags
     * @param name Detector name
     */
    void addObservation(const FM::Vec& z, double time, string flag = "", string name = "")
    {
      m_observations.push_back(observation_t(z, time, flag, name));
    }
    
    /**
     * Remove observations
     */
    void cleanup()
    {
      // clean current vectors
      m_observations.clear();
      m_assignments.clear();
    }
    
    /**
     * Size of the multitracker
     * @return Current number of filters
     */
    int size()
    {
      return m_filters.size();
    }
    
    /**
     * Return a particular filter of the multitracker
     * @param i Index of the filter
     * @return Reference to the filter
     */
    const filter_t& operator[](int i)
      {
	return m_filters[i];
      }
    
    /**
     * Perform prediction step for all the current filters
     * @param pm Prediction model
     */
    template<class PredictionModelType>
      void predict(PredictionModelType& pm)
      {
	typename std::vector<filter_t>::iterator fi, fiEnd = m_filters.end();
	for (fi = m_filters.begin(); fi != fiEnd; fi++) {
	  fi->filter->predict(pm);
	}
      }
    
    /**
     * Perform data association and update step for all the current filters, create new ones and remove those which are no more necessary
     * @param om Observation model
     * @param om_flag Observation model flag (CARTESIAN or POLAR)
     * @param alg Data association algorithm (NN or NNJPDA)
     * @param seqSize Minimum number of observations necessary for new track creation
     * @param seqTime Minimum interval between observations for new track creation
     * @param stdLimit Upper limit for the standard deviation of the estimated position
     */
    template<class ObservationModelType>
      void process(ObservationModelType& om, observ_model_t om_flag = CARTESIAN, association_t alg = NN, unsigned int seqSize = 5, double seqTime = 0.2, double stdLimit = 1.0)
      {
	// data association
	if (dataAssociation(om, alg)) {
	  // update
	  observe(om);
	}
	pruneTracks(stdLimit);
	if (m_observations.size())
	  createTracks(om, om_flag, seqSize, seqTime);
	// finished
	cleanup();
      }
    
    /**
     * Returns the vector of indexes of unmatched observations
     */
    const std::vector<size_t>* getUnmatched()
    {
      return &m_unmatched;
    }
    
    /**
     * Print state and covariance of all the current filters
     */
    void print()
    {
      int i = 0;
      typename std::vector<filter_t>::iterator fi, fiEnd = m_filters.end();
      for (fi = m_filters.begin(); fi != fiEnd; fi++) {
	cout << "Filter[" << i++ << "]\n\tx = " << fi->filter->x << "\n\tX = " << fi->filter->X << endl;
      }
    }
    
  private:
    void addFilter(const FM::Vec& initState, const FM::SymMatrix& initCov)
    {
      FilterType* filter = new FilterType(xSize);
      filter.init(initState, initCov);
      addFilter(filter);
    }
    
    void addFilter(FilterType* filter)
    {
      filter_t f = {m_filterNum++, filter};
      m_filters.push_back(f);
    }
    
    template<class ObservationModelType>
      bool dataAssociation(ObservationModelType& om, association_t alg = NN)
      {
	const size_t M = m_observations.size(), N = m_filters.size();
	
	if (M == 0) // no observation, do nothing
	  return false;
	
	if (N != 0) { // observations and tracks, associate
	  jpda::JPDA* jpda;
	  vector< size_t > znum; // this would contain the number of observations for each sensor
	  if (alg == NNJPDA) { /// NNJPDA data association (one sensor)
	    znum.push_back(M); // only one in this case
	    jpda = new jpda::JPDA(znum, N);
	  }
	  
	  AssociationMatrix amat(M, N);
	  int dim = om.z_size;
	  Vec zp(dim), s(dim);
	  SymMatrix Zp(dim, dim), S(dim, dim);
	  for (int j = 0; j < N; j++) {
	    m_filters[j].filter->predict_observation(om, zp, Zp);
	    S = Zp + om.Z; // H*P*H' + R
	    for (int i = 0; i < M; i++) {
	      s = zp - m_observations[i].vec;
	      om.normalise(s, zp);
	      try {
		if (AM::mahalanobis(s, S) > AM::gate(s.size())) {
		  amat[i][j] = DBL_MAX; // gating
		}
		else {
		  amat[i][j] = AM::correlation_log(s, S);
		  if (alg == NNJPDA) {
		    jpda->Omega[0][i][j+1] = true;
		    jpda->Lambda[0][i][j+1] = jpda::logGauss(s, S);
		  }
		}
	      } catch (Bayesian_filter::Filter_exception& e) {
		cerr << "###### Exception in AssociationMatrix #####\n";
		cerr << "Message: " << e.what() << endl;
		amat[i][j] = DBL_MAX;  // set to maximum
	      }
	    }
	  }
	  
	  if (alg == NN) { /// NN data association
	    amat.computeNN(CORRELATION_LOG);
	    // record unmatched observations for possible candidates creation
	    m_unmatched = amat.URow;
	    // data assignment
	    for (size_t n = 0; n < amat.NN.size(); n++) {
	      m_assignments.insert(std::make_pair(amat.NN[n].row, amat.NN[n].col));
	    }
	  }
	  else if (alg == NNJPDA) { /// NNJPDA data association (one sensor)
	    // compute associations
	    jpda->getAssociations();
	    jpda->getProbabilities();
	    vector< jpda::Association > association(znum.size());
	    jpda->getMultiNNJPDA(association);
	    // jpda->getMonoNNJPDA(association);
	    // data assignment
	    jpda::Association::iterator ai, aiEnd = association[0].end();
	    for (ai = association[0].begin(); ai != aiEnd; ai++) {
	      if (ai->t) { // not a clutter
		m_assignments.insert(std::make_pair(ai->z, ai->t - 1));
	      }
	      else { // add clutter to unmatched list
		m_unmatched.push_back(ai->z);
	      }
	    }
	    delete jpda;
	  }
	  else {
	    cerr << "###### Unknown association algorithm: " << alg << " #####\n";
	    return false;
	  }
	  
	  return true;
	}
	
	for (int i = 0; i < M; i++) // simply record unmatched
	  m_unmatched.push_back(i);
	
	return false;
      }
    
    void pruneTracks(double stdLimit = 1.0)
    {
      // remove lost tracks
      typename std::vector<filter_t>::iterator fi = m_filters.begin(), fiEnd = m_filters.end();
      while (fi != fiEnd) {
	if (isLost(fi->filter, stdLimit)) {
	  delete fi->filter;
	  fi = m_filters.erase(fi);
	  fiEnd = m_filters.end();
	}
	else {
	  fi++;
	}
      }
    }
    
    // seqSize = Minimum number of unmatched observations to create new track hypothesis
    // seqTime = Maximum time interval between these observations
    template<class ObservationModelType>
      void createTracks(ObservationModelType& om, observ_model_t om_flag, unsigned int seqSize, double seqTime)
      {
	// create new tracks from unmatched observations
	std::vector<size_t>::iterator ui = m_unmatched.begin();
	while (ui != m_unmatched.end()) {
	  std::vector<sequence_t>::iterator si = m_sequences.begin();
	  bool matched = false;
	  while (si != m_sequences.end()) {
	    if (m_observations[*ui].time - si->back().time > seqTime) { // erase old unmatched observations
	      si = m_sequences.erase(si);
	    }
	    else if (AM::mahalanobis(m_observations[*ui].vec, om.Z, si->back().vec, om.Z) <= AM::gate(om.z_size)) { // observation close to a previous one
	      // add new track
	      si->push_back(m_observations[*ui]);
	      FilterType* filter;
	      if (si->size() >= seqSize && initialize(filter, *si, om_flag)) {  // there's a minimum number of sequential observations
		addFilter(filter);
#ifdef OL
		m_filters.back().pose_id = si->back().flag;
		m_filters.back().detector_name = si->back().name;
#endif
		// remove sequence
		si = m_sequences.erase(si);
		matched = true;
	      }
	      else {
		si++;
	      }
	    }
	    else {
	      si++;
	    }
	  }
	  if (matched) {
	    // remove from unmatched list
	    ui = m_unmatched.erase(ui);
	  }
	  else {
	    ui++;
	  }
	}
	// memorize remaining unmatched observations
	std::vector<size_t>::iterator uiEnd = m_unmatched.end();
	for (ui = m_unmatched.begin() ; ui != uiEnd; ui++) {
	  sequence_t s;
	  s.push_back(m_observations[*ui]);
	  m_sequences.push_back(s);
	}
	// reset vector of (indexes of) unmatched observations
	m_unmatched.clear();
      }
    
    template<class ObservationModelType>
      void observe(ObservationModelType& om)
      {
	typename std::map<int, int>::iterator ai, aiEnd = m_assignments.end();
	for (ai = m_assignments.begin(); ai != aiEnd; ai++) {
	  m_filters[ai->second].filter->observe(om, m_observations[ai->first].vec);
#ifdef OL
	  m_filters[ai->second].pose_id = m_observations[ai->first].flag;
	  m_filters[ai->second].detector_name = m_observations[ai->first].name;
#endif
	}
      }
  };
} // namespace MTRK

#endif
