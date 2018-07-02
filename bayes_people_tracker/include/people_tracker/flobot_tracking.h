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

#ifndef SIMPLE_TRACKING_H
#define SIMPLE_TRACKING_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <bayes_tracking/multitracker.h>
#include <bayes_tracking/models.h>
#include <bayes_tracking/ekfilter.h>
#include <bayes_tracking/ukfilter.h>
#include <bayes_tracking/pfilter.h>
#include <cstdio>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/optional.hpp>
#include <math.h>

using namespace std;
using namespace MTRK;
using namespace Models;

// rule to detect lost track
template<class FilterType>
bool MTRK::isLost(const FilterType* filter, double stdLimit) {
  //ROS_INFO("var_x: %f, var_y: %f",filter->X(0,0), filter->X(2,2));
  // track lost if var(x)+var(y) > stdLimit^2
  if(filter->X(0,0) + filter->X(2,2) > sqr(stdLimit))
    return true;
  return false;
}

// rule to create new track
template<class FilterType>
bool MTRK::initialize(FilterType* &filter, sequence_t& obsvSeq, observ_model_t om_flag) {
  assert(obsvSeq.size());
  
  if(om_flag == CARTESIAN) {
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    if(dt == 0) return false;
    //assert(dt); // dt must not be null
    
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
  
  if(om_flag == POLAR) {    
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    if(dt == 0) return false;
    //assert(dt); // dt must not be null
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

template<typename FilterType>
class SimpleTracking {
 public:
  SimpleTracking(double sLimit = 1.0) {
    time = getTime();
    observation = new FM::Vec(2);
    stdLimit = sLimit;
  }
  
  void createConstantVelocityModel(double vel_noise_x, double vel_noise_y) {
    cvm = new CVModel(vel_noise_x, vel_noise_y);
  }
  
  void addDetectorModel(std::string name, association_t alg, observ_model_t om_flag, double pos_noise_x, double pos_noise_y, unsigned int seqSize = 5, double seqTime = 0.2) {
    ROS_INFO("Adding detector model for: %s.", name.c_str());
    detector_model det;
    det.om_flag = om_flag;
    det.alg = alg;
    if(om_flag == CARTESIAN)
      det.ctm = new CartesianModel(pos_noise_x, pos_noise_y);
    if(om_flag == POLAR)
      det.plm = new PolarModel(pos_noise_x, pos_noise_y);
    det.seqSize = seqSize;
    det.seqTime = seqTime;
    detectors[name] = det;
  }
  
  std::map<long, std::vector<geometry_msgs::Pose> > track(double* track_time = NULL) {
    boost::mutex::scoped_lock lock(mutex);
    std::map<long, std::vector<geometry_msgs::Pose> > result;
    dt = getTime() - time;
    time += dt;
    if(track_time) *track_time = time;
    
    // prediction
    cvm->update(dt);
    mtrk.template predict<CVModel>(*cvm);
    
    //=========================== @todo what's this? ===========================//
    detector_model dummy_det;
    mtrk.process(*(dummy_det.ctm));
    //
    //        for(typename std::map<std::string, detector_model>::const_iterator it = detectors.begin();
    //            it != detectors.end();
    //            ++it) {
    //            // process observations (if available) and update tracks
    //            mtrk.process(*(it->second.ctm), it->second.alg,  it->second.seqSize,  it->second.seqTime);
    //        }
    //==========================================================================//
    
    for (int i = 0; i < mtrk.size(); i++) {
      double theta = atan2(mtrk[i].filter->x[3], mtrk[i].filter->x[1]);
      /* ROS_DEBUG("trk_%ld: Position: (%f, %f), Orientation: %f, Std Deviation: %f, %f", */
      /* 		mtrk[i].id, */
      /* 		mtrk[i].filter->x[0], mtrk[i].filter->x[2], //x, y */
      /* 		theta, //orientation */
      /* 		sqrt(mtrk[i].filter->X(0,0)), sqrt(mtrk[i].filter->X(2,2))//std dev */
      /* 		); */
      
      geometry_msgs::Pose pose, vel, var; // position, velocity, variance
      
      pose.position.x = mtrk[i].filter->x[0];
      pose.position.y = mtrk[i].filter->x[2];
      pose.orientation.z = sin(theta/2);
      pose.orientation.w = cos(theta/2);
#ifdef ONLINE_LEARNING
      if(mtrk[i].detector_name == "object3d_detector")
	pose.position.z = std::atoi(mtrk[i].pose_id.c_str());
      else
	pose.position.z = -1.0;
      //std::cerr << "track ID = " << mtrk[i].id << ", pose ID = " << mtrk[i].pose_id << std::endl;
#endif
      result[mtrk[i].id].push_back(pose);
      
      vel.position.x = mtrk[i].filter->x[1];
      vel.position.y = mtrk[i].filter->x[3];
      result[mtrk[i].id].push_back(vel);
      
      var.position.x = mtrk[i].filter->X(0,0);
      var.position.y = mtrk[i].filter->X(2,2);
      var.orientation.x = mtrk[i].filter->X(0,2);
      var.orientation.y = mtrk[i].filter->X(2,0);
      result[mtrk[i].id].push_back(var);
    }
    return result;
  }
  
  void addObservation(std::string detector_name, std::vector<geometry_msgs::Point> obsv, double obsv_time, geometry_msgs::Pose &robot_pose) {
    boost::mutex::scoped_lock lock(mutex);
    ROS_DEBUG("Adding new observations for detector: %s", detector_name.c_str());
    // add last observation/s to tracker
    detector_model det;
    try {
      det = detectors.at(detector_name);
    } catch (std::out_of_range &exc) {
      ROS_ERROR("Detector %s was not registered!", detector_name.c_str());
      return;
    }
    
    dt = getTime() - time;
    time += dt;
    
    // prediction
    cvm->update(dt);
    mtrk.template predict<CVModel>(*cvm);
    
    // mtrk.process(*(det.ctm), det.alg); @todo can we remove this?
    
    for(std::vector<geometry_msgs::Point>::iterator li = obsv.begin(); li != obsv.end(); ++li) {
      if(det.om_flag == CARTESIAN) {
	(*observation)[0] = li->x;
	(*observation)[1] = li->y;
      }
      if(det.om_flag == POLAR) {
	(*observation)[0] = atan2(li->y, li->x); // bearing
	(*observation)[1] = sqrt(pow(li->x,2) + pow(li->y,2)); // range
      }
#ifdef ONLINE_LEARNING
      // @todo online learning ros message type
      mtrk.addObservation(*observation, obsv_time, boost::to_string((uint32_t)li->z), detector_name);
#else
      mtrk.addObservation(*observation, obsv_time);
#endif
    }
    
    if(det.om_flag == CARTESIAN) {
      mtrk.process(*(det.ctm), det.om_flag, det.alg, det.seqSize, det.seqTime, stdLimit);
    }
    if(det.om_flag == POLAR) {
      //det.plm->update(robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.w);
      det.plm->update(0, 0, 0);
      mtrk.process(*(det.plm), det.om_flag, det.alg, det.seqSize, det.seqTime, stdLimit);
    }
  }
  
 private:
  FM::Vec *observation;             // observation [x, y]
  double dt, time;
  boost::mutex mutex;
  CVModel *cvm;                     // CV model
  MultiTracker<FilterType, 4> mtrk; // state [x, v_x, y, v_y]
  double stdLimit;                  // upper limit for the variance of estimation position
  
  typedef struct {
    CartesianModel *ctm;    // Cartesian observation model
    PolarModel *plm;        // Polar observation model
    observ_model_t om_flag; // Observation model flag
    association_t alg;      // Data association algorithm
    unsigned int seqSize;   // Minimum number of observations for new track creation
    double seqTime;         // Minimum interval between observations for new track creation
  } detector_model;
  std::map<std::string, detector_model> detectors;
  
  double getTime() {
    return ros::Time::now().toSec();
  }
};
#endif //SIMPLE_TRACKING_H
