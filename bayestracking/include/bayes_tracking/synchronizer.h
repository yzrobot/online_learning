/***************************************************************************
 *   Copyright (C) 2007 by Nicola Bellotto   *
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
#ifndef SYNCHRONIZER_H
#define SYNCHRONIZER_H

#include <iostream>
#include <vector>
#include <deque>
#include <cstdlib>
#include <string>
#include <fstream>

using namespace std;

/**
*	@author Nicola Bellotto <nbello@essex.ac.uk>
*  Class to synchronize data from odometry, sonar, laser and camera
*/
template <class odometry_t, class sonar_t, class laser_t, class camera_t >
class Synchronizer {

   typedef pair< odometry_t, double > odometry_pair;
   typedef pair< sonar_t, double > sonar_pair;
   typedef pair< laser_t, double > laser_pair;
   typedef pair< camera_t, double > camera_pair;

private:
   deque< odometry_pair > m_odometryQueue;
   deque< sonar_pair > m_sonarQueue;
   deque< laser_pair > m_laserQueue;
   deque< camera_pair > m_cameraQueue;
   
   double m_odometryTimeStamp;
   double m_sonarTimeStamp;
   double m_laserTimeStamp;
   double m_cameraTimeStamp;
   
   double m_maxTimeGap;
   double m_syncTime;
   
   odometry_pair m_syncOdometry;
   sonar_pair m_syncSonar;
   laser_pair m_syncLaser;
   camera_pair m_syncCamera;

public:

   /**
    * Constructor
    */
   Synchronizer(double maxTimeGap) {
      m_maxTimeGap = maxTimeGap;
      reset();
   }

   /**
    * Destructor
    */
   ~Synchronizer() {}
   
   /**
    * Return current synchronization time
    * @return Synchronization time
    */
   double getSyncTime() { return m_syncTime; }
    
   /**
    * Get synchronized odometry data
    * @return Data
    */
   odometry_t getOdometryData() { return m_syncOdometry.first; }
   
   /**
    * Get timestamp of synchronized odometry data
    * @return Timestamp
    */
   double getOdometryTime() { return m_syncOdometry.second; }
   
   /**
    * Get synchronized sonar data
    * @return Data
    */
   sonar_t getSonarData() { return m_syncSonar.first; }
   
   /**
    * Get timestamp of synchronized sonar data
    * @return Timestamp
    */
   double getSonarTime() { return m_syncSonar.second; }
   
   /**
    * Get synchronized laser data
    * @return Data
    */
   laser_t getLaserData() { return m_syncLaser.first; }
   
   /**
    * Get timestamp of synchronized laser data
    * @return Timestamp
    */
   double getLaserTime() { return m_syncLaser.second; }
   
   /**
    * Get synchronized camera data
    * @return Data
    */
   camera_t getCameraData() { return m_syncCamera.first; }
   
   /**
    * Get timestamp of synchronized camera data
    * @return Timestamp
    */
   double getCameraTime() { return m_syncCamera.second; }

   /**
    * Add new odometry data to the queue
    * @param data Data
    * @param timeStamp Timestamp
    * @return False if new timestamp <= current timestamp
    */
   bool addOdometry(odometry_t data, double timeStamp) {
      if (timeStamp > m_odometryTimeStamp)
      {  // queue new odometry with timestamp
         m_odometryQueue.push_front(make_pair(data, timeStamp));
         m_odometryTimeStamp = timeStamp;
         return true;
      }
      return false;
   }

   /**
    * Add new sonar data to the queue
    * @param data Data
    * @param timeStamp Timestamp
    * @return False if new timestamp <= current timestamp
    */
   bool addSonar(sonar_t data, double timeStamp) {
      if (timeStamp > m_sonarTimeStamp)
      {  // queue new frames with timestamp
         m_sonarQueue.push_front(make_pair(data, timeStamp));
         m_sonarTimeStamp = timeStamp;
         return true;
      }
      return false;
   }

   /**
    * Add new laser data to the queue
    * @param data Data
    * @param timeStamp Timestamp
    * @return False if new timestamp <= current timestamp
    */
   bool addLaser(laser_t data, double timeStamp) {
      if (timeStamp > m_laserTimeStamp)
      {  // queue new laser data with timestamp
         m_laserQueue.push_front(make_pair(data, timeStamp));
         m_laserTimeStamp = timeStamp;
         return true;
      }
      return false;
   }

   /**
    * Add new camera data to the queue
    * @param data Data
    * @param timeStamp Timestamp
    * @return False if new timestamp <= current timestamp
    */
   bool addCamera(camera_t data, double timeStamp) {
      if (timeStamp > m_cameraTimeStamp)
      {  // queue new frames with timestamp
         m_cameraQueue.push_front(make_pair(data, timeStamp));
         m_cameraTimeStamp = timeStamp;
         return true;
      }
      return false;
   }
   
   /**
    * Synchronize data
    * @return False if one or more empty queues, or if timestamps \
    * are too different to synchronize data
    */
   bool synchronize() {
      // get the minimum timestamp as reference
      double ref_timestamp =
         std::min(std::min(m_odometryTimeStamp, m_sonarTimeStamp),
                  std::min(m_laserTimeStamp, m_cameraTimeStamp));
      if (ref_timestamp == m_syncTime) {
         return false;
      }
      else {
         m_syncTime = ref_timestamp;
      }
      
      if (m_odometryQueue.empty() ||
          m_sonarQueue.empty() ||
          m_laserQueue.empty() ||
          m_cameraQueue.empty()) {
         return false;   // not enough data, go back to get new
      }
         
      bool noReading = false;
      // retrieve odometry with timestamp similar to reference
      typename deque< odometry_pair >::iterator
         oi = m_odometryQueue.begin(), prev_oi = oi;
      while ((oi != m_odometryQueue.end()) && (oi->second > ref_timestamp))
         prev_oi = oi++;
      {
         double dt2 = prev_oi->second - ref_timestamp;
         if (oi != m_odometryQueue.end()) {
            double dt1 = ref_timestamp - oi->second;
            if ((dt1 < dt2) && (dt1 <= m_maxTimeGap)) {
               m_syncOdometry = *oi;
               m_odometryQueue.erase(oi, m_odometryQueue.end());
            }
            else if (dt2 <= m_maxTimeGap) {
               m_syncOdometry = *prev_oi;
               m_odometryQueue.erase(prev_oi, m_odometryQueue.end());
            }
            else {
               noReading = true;
               m_odometryQueue.erase(oi, m_odometryQueue.end());
            }
         }
         else if (dt2 <= m_maxTimeGap) {
            m_syncOdometry = *prev_oi;
//             m_odometryQueue.erase(prev_oi, m_odometryQueue.end());
         }
         else {
            noReading = true;
         }
      }
      // retrieve sonar with timestamp similar to reference
      typename deque< sonar_pair >::iterator
         si = m_sonarQueue.begin(), prev_si = si;
      while ((si != m_sonarQueue.end()) && (si->second > ref_timestamp))
         prev_si = si++;
      {
         double dt2 = prev_si->second - ref_timestamp;
         if (si != m_sonarQueue.end()) {
            double dt1 = ref_timestamp - si->second;
            if ((dt1 < dt2) && (dt1 <= m_maxTimeGap)) {
               m_syncSonar = *si;
               m_sonarQueue.erase(si, m_sonarQueue.end());
            }
            else if (dt2 <= m_maxTimeGap) {
               m_syncSonar = *prev_si;
               m_sonarQueue.erase(prev_si, m_sonarQueue.end());
            }
            else {
               noReading = true;
               m_sonarQueue.erase(si, m_sonarQueue.end());
            }
         }
         else if (dt2 <= m_maxTimeGap) {
            m_syncSonar = *prev_si;
//             m_sonarQueue.erase(prev_si, m_sonarQueue.end());
         }
         else {
            noReading = true;
         }
      }
      // retrieve laser with timestamp similar to reference
      typename deque< laser_pair >::iterator
         li = m_laserQueue.begin(), prev_li = li;
      while ((li != m_laserQueue.end()) && (li->second > ref_timestamp))
         prev_li = li++;
      {
         double dt2 = prev_li->second - ref_timestamp;
         if (li != m_laserQueue.end()) {
            double dt1 = ref_timestamp - li->second;
            if ((dt1 < dt2) && (dt1 <= m_maxTimeGap)) {
               m_syncLaser = *li;
               m_laserQueue.erase(li, m_laserQueue.end());
            }
            else if (dt2 <= m_maxTimeGap) {
               m_syncLaser = *prev_li;
               m_laserQueue.erase(prev_li, m_laserQueue.end());
            }
            else {
               noReading = true;
               m_laserQueue.erase(li, m_laserQueue.end());
            }
         }
         else if (dt2 <= m_maxTimeGap) {
            m_syncLaser = *prev_li;
//             m_laserQueue.erase(prev_li, m_laserQueue.end());
         }
         else {
            noReading = true;
         }
      }
      // retrieve frame with timestamp similar to reference
      typename deque< camera_pair >::iterator
         ci = m_cameraQueue.begin(), prev_ci = ci;
      while ((ci != m_cameraQueue.end()) && (ci->second > ref_timestamp))
         prev_ci = ci++;
      {
         double dt2 = prev_ci->second - ref_timestamp;
         if (ci != m_cameraQueue.end()) {
            double dt1 = ref_timestamp - ci->second;
            if ((dt1 < dt2) && (dt1 <= m_maxTimeGap)) {
               m_syncCamera = *ci;
               m_cameraQueue.erase(ci, m_cameraQueue.end());
            }
            else if (dt2 <= m_maxTimeGap) {
               m_syncCamera = *prev_ci;
               m_cameraQueue.erase(prev_ci, m_cameraQueue.end());
            }
            else {
               noReading = true;
               m_cameraQueue.erase(ci, m_cameraQueue.end());
            }
         }
         else if (dt2 <= m_maxTimeGap) {
            m_syncCamera = *prev_ci;
//             m_cameraQueue.erase(prev_ci, m_cameraQueue.end());
         }
         else {
            noReading = true;
         }
      }
      
      if (noReading)
         return false;
         
      return true;
   }
   
   /**
    * Reset data queues
    */
   void reset() {
      // clear queues
      m_odometryQueue.clear();
      m_sonarQueue.clear();
      m_laserQueue.clear();
      m_cameraQueue.clear();
      // reset timestamps
      m_odometryTimeStamp = -1.;
      m_sonarTimeStamp = -1.;
      m_laserTimeStamp = -1.;
      m_cameraTimeStamp = -1.;
      m_syncTime = -1.;
   }
};

#endif
