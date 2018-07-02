#ifndef PEDESTRIANLOCALISATION_H
#define PEDESTRIANLOCALISATION_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <bayes_tracking/BayesFilter/bayesFlt.hpp>

#include <XmlRpcValue.h>

#include <string.h>
#include <vector>
#include <math.h>

#include "bayes_people_tracker/PeopleTracker.h"

#include "people_tracker/flobot_tracking.h"
#include "people_tracker/asso_exception.h"

#define INVALID_ID -1

class PeopleTracker
{
 public:
  PeopleTracker();
  
 private:
  void trackingThread();
  void publishDetections(bayes_people_tracker::PeopleTracker msg);
  void publishDetections(geometry_msgs::PoseStamped msg);
  void publishDetections(geometry_msgs::PoseArray msg);
  void publishDetections(people_msgs::People msg);
  void publishDetections(double time_sec,
			 geometry_msgs::Pose closest,
			 std::vector<geometry_msgs::Pose> ppl,
			 std::vector<geometry_msgs::Pose> vels,
			 std::vector<std::string> uuids,
			 std::vector<double> distances,
			 std::vector<double> angles,
			 double min_dist,
			 double angle);
  void publishTrajectory(std::vector<geometry_msgs::Pose> poses,
			 std::vector<geometry_msgs::Pose> vels,
			 std::vector<geometry_msgs::Pose> vars,
			 std::vector<long> pids,
			 ros::Publisher& pub);
  void PN_experts(geometry_msgs::PoseArray &variance,
		  geometry_msgs::PoseArray &velocity,
		  geometry_msgs::PoseArray &trajectory);
  void createVisualisation(std::vector<geometry_msgs::Pose> poses,
			   std::vector<long> pids,
			   ros::Publisher& pub);
  std::vector<double> cartesianToPolar(geometry_msgs::Point point);
  void detectorCallback(const geometry_msgs::PoseArray::ConstPtr &pta, string detector);
  void connectCallback(ros::NodeHandle &n);
  void parseParams(ros::NodeHandle);
  
  std::string generateUUID(std::string time, long id) {
    boost::uuids::name_generator gen(dns_namespace_uuid);
    time += num_to_str<long>(id);
    
    return num_to_str<boost::uuids::uuid>(gen(time.c_str()));
  }
  
  geometry_msgs::Point generate_position(geometry_msgs::Point centre, double angle, double dx, double dy)
    {
      float s = sin(angle);
      float c = cos(angle);
      
      // rotate point
      geometry_msgs::Point res;
      res.x = dx * c - dy * s;
      res.y = dx * s + dy * c;
      
      // translate point back:
      res.x += centre.x;
      res.y += centre.y;
      res.z  = centre.z;
      return res;
    }
  
  geometry_msgs::Pose generate_extremity_position(geometry_msgs::Pose centre, double dx, double dy, double z) {
    double angle = tf::getYaw(centre.orientation) + M_PI/2;
        geometry_msgs::Point p = centre.position;
        p.z = z;
        centre.position = generate_position(p, angle, dx, dy);
        return centre;
  }
  
  visualization_msgs::Marker createMarker(
					  int id,
					  int type,
					  int action,
					  geometry_msgs::Pose pose,
					  geometry_msgs::Vector3 scale,
					  std_msgs::ColorRGBA color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = ++marker_seq;
    marker.ns = "people_tracker";
    marker.id = id;
    marker.type = type;
    marker.action = action;
    marker.pose = pose;
    marker.scale = scale;
    marker.color = color;
    marker.lifetime = ros::Duration(1);
    return marker;
  }
  
  visualization_msgs::Marker createHead(
					int id,
					int action,
					geometry_msgs::Pose pose) {
    geometry_msgs::Vector3 scale;
    scale.x = 0.3;
    scale.y = 0.3;
    scale.z = 0.3;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 233.0F/255.0F;
    color.g = 150.0F/255.0F;
    color.b = 122.0F/255.0F;
    pose.position.z = 1.6;
    return createMarker(id, visualization_msgs::Marker::SPHERE, action, pose, scale, color);
  }
  
  visualization_msgs::Marker createBody(
					int id,
					int action,
					geometry_msgs::Pose pose) {
    geometry_msgs::Vector3 scale;
    scale.x = 0.35;
    scale.y = 0.35;
    scale.z = 0.7;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 139.0F/255.0F;
    color.g = 0.0F/255.0F;
    color.b = 0.0F/255.0F;
    pose.position.z = 1.1;
    return createMarker(id, visualization_msgs::Marker::CYLINDER, action, pose, scale, color);
  }
  
  std::vector<visualization_msgs::Marker> createLegs(
						     int idl, int idr,
						     int action,
						     geometry_msgs::Pose pose) {
    std::vector<visualization_msgs::Marker> legs;
    geometry_msgs::Vector3 scale;
    scale.x = 0.15;
    scale.y = 0.2;
    scale.z = 0.8;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0F/255.0F;
    color.g = 0.0F/255.0F;
    color.b = 139.0F/255.0F;
    legs.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, 0.1, 0.0, 0.4), scale, color));
    legs.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, -0.1, 0.0, 0.4), scale, color));
    return legs;
  }

  std::vector<visualization_msgs::Marker> createArms(
						     int idl, int idr,
						     int action,
						     geometry_msgs::Pose pose) {
    std::vector<visualization_msgs::Marker> arms;
    geometry_msgs::Vector3 scale;
    scale.x = 0.1;
    scale.y = 0.1;
    scale.z = 0.7;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 139.0F/255.0F;
    color.g = 0.0F/255.0F;
    color.b = 0.0F/255.0F;
    arms.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, 0.2, 0.0, 1.1), scale, color));
    arms.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, -0.2, 0.0, 1.1), scale, color));
    return arms;
  }
  
  std::vector<visualization_msgs::Marker> createHuman(
						      int id,
						      geometry_msgs::Pose pose) {
    std::vector<visualization_msgs::Marker> human;
    human.push_back(createHead(id++, visualization_msgs::Marker::ADD, pose));
    human.push_back(createBody(id++, visualization_msgs::Marker::ADD, pose));
    std::vector<visualization_msgs::Marker> legs = createLegs(id++, id++, visualization_msgs::Marker::ADD, pose);
    human.insert(human.end(), legs.begin(), legs.end());
    std::vector<visualization_msgs::Marker> arms = createArms(id++, id++, visualization_msgs::Marker::ADD, pose);
    human.insert(human.end(), arms.begin(), arms.end());
    return human;
  }
  
  template<typename T>
    std::string num_to_str(T num) {
    std::stringstream ss;
    ss << num;
    return ss.str();
  }
  
  ros::Publisher pub_detect;
  ros::Publisher pub_pose;
  ros::Publisher pub_pose_array;
  ros::Publisher pub_people;
  ros::Publisher pub_trajectory;
  ros::Publisher pub_trajectory_acc;
  ros::Publisher pub_marker;
  tf::TransformListener* listener;
  std::string target_frame;
  std::string base_link;
  double tracker_frequency;
  double human_path_min;
  double human_velo_min;
  double human_velo_max;
  double human_vari_max;
  double static_path_max;
  double static_velo_max;
  double static_vari_max;
  int repeated_poses_max;
  bool log_trajectories;
  bool publish_detections;
  unsigned long detect_seq;
  unsigned long marker_seq;
  double startup_time;
  std::string startup_time_str;
  
  boost::uuids::uuid dns_namespace_uuid;
  
  SimpleTracking<EKFilter> *ekf = NULL;
  SimpleTracking<UKFilter> *ukf = NULL;
  SimpleTracking<PFilter> *pf = NULL;
  std::map<std::pair<std::string, std::string>, ros::Subscriber> subscribers;
  std::vector<boost::tuple<long, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose> > previous_poses;
};

#endif // PEDESTRIANLOCALISATION_H
