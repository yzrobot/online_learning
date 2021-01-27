/***************************************************************************
 *   Copyright (C) 2011 by Nicola Bellotto (nbellotto@lincoln.ac.uk)       *
 *   Copyright (C) 2020 by Zhi Yan (zhi.yan@utbm.fr)                       *
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

#ifndef FLOBOT_TRACKING_H
#define FLOBOT_TRACKING_H

#include <ros/ros.h>
#include <people_msgs/Person.h>
#include <people_msgs/PositionMeasurementArray.h>

#include <bayes_tracking/multitracker.h>
#include <bayes_tracking/models.h>
#include <bayes_tracking/ekfilter.h>
#include <bayes_tracking/ukfilter.h>
#include <bayes_tracking/pfilter.h>

#include <boost/thread.hpp>

#define __APP_NAME__ "bayes_people_tracker"

using namespace std;
using namespace MTRK;
using namespace Models;

// rule to detect lost track
template<class FilterType>
bool MTRK::isLost(const FilterType* filter, double stdLimit) {
  //ROS_INFO("[%s] var_x: %f, var_y: %f", __APP_NAME__, filter->X(0,0), filter->X(2,2));
  if(filter->X(0,0) + filter->X(2,2) > sqr(stdLimit)) { // track lost if var(x)+var(y) > stdLimit^2
    return true;
  }
  return false;
}

// rule to create new track
template<class FilterType>
bool MTRK::initialize(FilterType* &filter, sequence_t& obsvSeq, observ_model_t om_flag) {
  assert(obsvSeq.size());
  
  if(om_flag == CARTESIAN) {
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    if(dt < DBL_EPSILON) {
      return false;
    }
    
    FM::Vec v((obsvSeq.back().vec - obsvSeq.front().vec) / dt);
    FM::Vec x(4);
    FM::SymMatrix X(4,4);
    
    x[0] = obsvSeq.back().vec[0];
    x[1] = v[0];
    x[2] = obsvSeq.back().vec[1];
    x[3] = v[1];
    X.clear();
    X(0,0) = sqr(0.2);
    X(1,1) = sqr(1.0);
    X(2,2) = sqr(0.2);
    X(3,3) = sqr(1.0);
    
    filter = new FilterType(4);
    filter->init(x, X);
  }
  else if(om_flag == POLAR) {    
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    if(dt < DBL_EPSILON) {
      return false;
    }
    
    double x2 = obsvSeq.back().vec[1] * cos(obsvSeq.back().vec[0]);
    double y2 = obsvSeq.back().vec[1] * sin(obsvSeq.back().vec[0]);
    double x1 = obsvSeq.front().vec[1] * cos(obsvSeq.front().vec[0]);
    double y1 = obsvSeq.front().vec[1] * sin(obsvSeq.front().vec[0]);
    
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

  if(om_flag == BEARING) return false;
  
  
  return true;
}

template<typename FilterType>
class SimpleTracking {
 public:
  SimpleTracking(double sLimit = 1.0) {
    time = ros::Time::now().toSec();
    observation = new FM::Vec(2);
    stdLimit = sLimit;
  }
  
  void createConstantVelocityModel(double vel_noise_x, double vel_noise_y) {
    cvm = new CVModel(vel_noise_x, vel_noise_y);
  }
  
  void addDetectorModel(std::string name, association_t alg, observ_model_t om_flag, double pos_noise_x, double pos_noise_y, unsigned int seqSize = 5, double seqTime = 0.2) {
    ROS_INFO("[%s] Adding detector model for: %s", __APP_NAME__, name.c_str());

    detector_model det;
    det.om_flag = om_flag;
    det.alg = alg;

    if(om_flag == CARTESIAN) {
      det.ctm = new CartesianModel(pos_noise_x, pos_noise_y);
    }
    else if(om_flag == POLAR) {
      det.plm = new PolarModel(pos_noise_x, pos_noise_y);
    }

    if(om_flag == BEARING)
      det.brm = new BearingModel(pos_noise_y); 
    
    det.seqSize = seqSize;
    det.seqTime = seqTime;
    detectors[name] = det;
  ROS_INFO_STREAM("Finished adding detector");

  }
  
  std::map<long, std::vector<people_msgs::Person> > track(double* track_time = NULL) {
    boost::mutex::scoped_lock lock(mutex);
    std::map<long, std::vector<people_msgs::Person> > result;
    dt = ros::Time::now().toSec() - time;
    time += dt;
    if(track_time) {
      *track_time = time;
    }
    
    // prediction
    cvm->update(dt);
    mtrk.template predict<CVModel>(*cvm);
    
    for(int i = 0; i < mtrk.size(); i++) {
      people_msgs::Person person, variance; // position, velocity, variance
      
      person.name = mtrk[i].detector;
      person.tags.push_back(mtrk[i].sampleID);
      person.reliability = mtrk[i].probability;
      
      person.position.x = mtrk[i].filter->x[0];
      person.position.y = mtrk[i].filter->x[2];
      
      person.velocity.x = mtrk[i].filter->x[1];
      person.velocity.y = mtrk[i].filter->x[3];

      result[mtrk[i].id].push_back(person);

      variance.position.x = mtrk[i].filter->X(0,0);
      variance.position.y = mtrk[i].filter->X(2,2);
      
      result[mtrk[i].id].push_back(variance);
    }
    
    return result;
  }

  void addObservation(std::string detector, people_msgs::PositionMeasurementArray &obsv, double obsv_time) {
    boost::mutex::scoped_lock lock(mutex);
    ROS_DEBUG("[%s] Adding new observations for detector: %s", __APP_NAME__, detector.c_str());
    
    // add last observation/s to tracker
    detector_model det;
    try {
      det = detectors.at(detector);
    } catch(std::out_of_range &exc) {
      ROS_ERROR("[%s] Detector %s was not registered!", __APP_NAME__, detector.c_str());
      return;
    }
    
    dt = ros::Time::now().toSec() - time;
    time += dt;
    
    // prediction
    cvm->update(dt);
    mtrk.template predict<CVModel>(*cvm);

    for(size_t i = 0; i < obsv.people.size(); i++) {
      if(det.om_flag == CARTESIAN) {
	(*observation)[0] = obsv.people[i].pos.x;
	(*observation)[1] = obsv.people[i].pos.y;
      }
      else if(det.om_flag == POLAR) {
	(*observation)[0] = atan2(obsv.people[i].pos.y, obsv.people[i].pos.x); // bearing
	(*observation)[1] = sqrt(pow(obsv.people[i].pos.x, 2) + pow(obsv.people[i].pos.y, 2)); // range
      }
      
      mtrk.addObservation(*observation, obsv_time, obsv.people[i].name, obsv.people[i].object_id, obsv.people[i].reliability);
    }
    
    if(det.om_flag == CARTESIAN) {
      mtrk.process(*(det.ctm), det.om_flag, det.alg, det.seqSize, det.seqTime, stdLimit);
    }
    else if(det.om_flag == POLAR) {
      mtrk.process(*(det.plm), det.om_flag, det.alg, det.seqSize, det.seqTime, stdLimit);
    }
  }
  
 private:
  FM::Vec *observation; // observation [x, y]
  double dt, time;
  boost::mutex mutex;
  CVModel *cvm; // Constant Velocity model
  MultiTracker<FilterType, 4> mtrk; // state [x, v_x, y, v_y]
  double stdLimit; // upper limit for the variance of estimation position
  
  typedef struct {
    CartesianModel *ctm;    // Cartesian observation model
    PolarModel *plm;        // Polar observation model
    BearingModel *brm;
    observ_model_t om_flag; // Observation model flag
    association_t alg;      // Data association algorithm
    unsigned int seqSize;   // Minimum number of observations for new track creation
    double seqTime;         // Minimum interval between observations for new track creation
  } detector_model;

  std::map<std::string, detector_model> detectors;
};
#endif //FLOBOT_TRACKING_H
