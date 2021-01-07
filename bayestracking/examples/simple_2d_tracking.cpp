/***************************************************************************
 *   Copyright (C) 2011 by Nicola Bellotto                                 *
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

#include "bayes_tracking/multitracker.h"
#include "bayes_tracking/models.h"
#include "bayes_tracking/ekfilter.h"
#include "bayes_tracking/ukfilter.h"
#include "bayes_tracking/pfilter.h"
#include <sys/time.h>
#include "bayes_tracking/trackwin.h"
#include <cstdio>

#define SLEEP_TIME 20000  // sleep time in [us]

using namespace std;
using namespace MTRK;
using namespace Models;

typedef UKFilter Filter;
const int numOfTarget = 3;	// x 2
const association_t alg = NNJPDA;
const observ_model_t om_flag = POLAR;

static double getTime()
{
  timeval t;
  gettimeofday(&t,NULL);
  return (double)t.tv_sec + (double)t.tv_usec/1000000.0;
}

// rule to detect lost track
template<class FilterType>
bool MTRK::isLost(const FilterType* filter, double varLimit = 1.0) {
  // track lost if var(x)+var(y) > varLimit
  if (filter->X(0,0) + filter->X(2,2) > sqr(varLimit))
    return true;
  return false;
}

// rule to create new track
template<class FilterType>
bool MTRK::initialize(FilterType* &filter, sequence_t& obsvSeq, observ_model_t om_flag) {
  assert(obsvSeq.size());
  
  if (om_flag == CARTESIAN) {
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    assert(dt); // dt must not be null
    FM::Vec v((obsvSeq.back().vec - obsvSeq.front().vec) / dt);
    
    FM::Vec x(4);
    FM::SymMatrix X(4,4);
    
    x[0] = obsvSeq.back().vec[0];
    x[1] = v[0];
    x[2] = obsvSeq.back().vec[1];
    x[3] = v[1];
    X.clear();
    X(0,0) = sqr(0.5);
    X(1,1) = sqr(1.5);
    X(2,2) = sqr(0.5);
    X(3,3) = sqr(1.5);
    
    filter = new FilterType(4);
    filter->init(x, X);
  }
  
  if (om_flag == POLAR) {
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    assert(dt); // dt must not be null
    double x2 = obsvSeq.back().vec[1]*cos(obsvSeq.back().vec[0]);
    double y2 = obsvSeq.back().vec[1]*sin(obsvSeq.back().vec[0]);
    double x1 = obsvSeq.front().vec[1]*cos(obsvSeq.front().vec[0]);
    double y1 = obsvSeq.front().vec[1]*sin(obsvSeq.front().vec[0]);
    
    FM::Vec x(4);
    FM::SymMatrix X(4,4);
    
    x[0] = x2;
    x[1] = (x2-x1)/dt;
    x[2] = y2;
    x[3] = (y2-y1)/dt;
    X.clear();
    X(0,0) = sqr(0.5);
    X(1,1) = sqr(1.5);
    X(2,2) = sqr(0.5);
    X(3,3) = sqr(1.5);
    
    filter = new FilterType(4);
    filter->init(x, X);
  }
  return true;
}

// command line option
bool textDebug = false;
bool winDebug = false;
bool sendPredictions = false;

int main(int argc, char *argv[]) {
  cout.precision(20);
  
  // parse arguments
  for (int c = 1; c < argc; c++) {
    if (strcmp(argv[c], "-g") == 0 ) {
      textDebug = true;
      continue;
    }
    if (strcmp(argv[c], "-w") == 0 ) {
      winDebug = true;
      continue;
    }
    cerr << "Unknown option " << argv[c] << endl;
    cerr << "Valid options are '-g' (text debug) and '-w' (visual debug)" << endl;
    exit(EXIT_FAILURE);
  }
  
  MultiTracker<Filter, 4> mtrk;    // state [x, v_x, y, v_y]
  CVModel cvm(5.0, 5.0);           // CV model with sigma_x = sigma_y = 5.0
  CartesianModel ctm(1.0, 1.0);    // Cartesian observation model
  PolarModel plm(0.3, 1.0);        // Polar observation model
  FM::Vec observation(2);          // observation [x, y]
  vector<FM::Vec> obsvBuffer;      // observation buffer
  
  double dt, time = getTime(), obsvTime = -1;
  
  TrackWin* trkwin = 0;
  if (winDebug) {
    trkwin = new TrackWin(__FILE__, 800, 600, 30.);
    trkwin->setOrigin(0., 0., 0.);
    trkwin->create();
  }
  char label[256];
  
  for (;;) {
    ///////////////// estimation begin /////////////////
    //     #warning fix 'dt' if not debugging
    dt = /*20e-3;*/getTime() - time;
    time += dt;
    
    // prediction
    cvm.update(dt);
    mtrk.predict<CVModel>(cvm);
    
    // observation
    obsvBuffer.clear();
    if (time - obsvTime > 30e-3) {
      obsvTime = time;
      for (int i = 0; i < numOfTarget; i++) {
        FM::Vec obsv(2);
	double range = 3 * (i+1);
	double angle = time / (i+1);
        // counter-clockwise 
	if (om_flag == CARTESIAN) {
	  obsv[0] = range * cos(angle);
	  obsv[1] = range * sin(angle);
	  obsvBuffer.push_back(obsv);
	  // clockwise
	  obsv[0] = range * cos(-angle);
	  obsv[1] = range * sin(-angle);
	  obsvBuffer.push_back(obsv);
	}
	if (om_flag == POLAR) {
	  obsv[0] = angle;
	  obsv[1] = range;
	  obsvBuffer.push_back(obsv);
	  // clockwise
	  obsv[0] = -angle;
	  obsv[1] = range;
	  obsvBuffer.push_back(obsv);
	}
	// put some false positives
        if (rand() > 0.95*RAND_MAX) {
          obsv[0] *= 2.0 * rand() / RAND_MAX;
          obsv[1] *= 3.0 * rand() / RAND_MAX;
          obsvBuffer.push_back(obsv);
        }
      }
      
      // add last observation/s to tracker
      vector<FM::Vec>::iterator li, liEnd = obsvBuffer.end();
      for (li = obsvBuffer.begin(); li != liEnd; li++) {
        observation[0] = (*li)[0];
        observation[1] = (*li)[1];
        mtrk.addObservation(observation, obsvTime);
	
        if (winDebug) {
          // show on trackwin
          sprintf(label, "(%f,%f)", observation[0], observation[1]);
	  if(om_flag == CARTESIAN)
	    trkwin->setObject(label, observation[0], observation[1], TrackWin::NO_ORIENTATION, TrackWin::GREY);
	  if(om_flag == POLAR)
	    trkwin->setObject(label, observation[1]*cos(observation[0]), observation[1]*sin(observation[0]), TrackWin::NO_ORIENTATION, TrackWin::GREY);
	}
      }
      
      // process observations (if available) and update tracks
      if (om_flag == CARTESIAN) {
	mtrk.process(ctm, CARTESIAN, alg);
      }
      if (om_flag == POLAR) {
	plm.update(0, 0, 0);
	mtrk.process(plm, POLAR, alg);
      }
      //       mtrk.print();
    }
    ///////////////// estimation end /////////////////
    
    // show on trackwin
    if (winDebug) {
      int n = mtrk.size();
      for (int i = 0; i < n; i++) {
        sprintf(label, "trk_%ld", mtrk[i].id);
        trkwin->setObject(label, mtrk[i].filter->x[0], mtrk[i].filter->x[2], atan2(mtrk[i].filter->x[3], mtrk[i].filter->x[1])/*orientation*/,
			  TrackWin::RED, mtrk[i].filter->X(0,0), mtrk[i].filter->X(2,2), mtrk[i].filter->X(1,2)/*(co)variance*/);
      }
      trkwin->update(20);
    }
    
    int slp = SLEEP_TIME - (int)(1e6 * (getTime() - time));
    usleep(slp > 0 ? slp : 0);
  }
  
  if (winDebug) {
    trkwin->destroy();
    delete trkwin;
  }
  
  return EXIT_SUCCESS;
}
