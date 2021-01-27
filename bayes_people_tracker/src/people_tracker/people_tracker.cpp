#include "people_tracker/people_tracker.h"

#define UNKNOWN -1
#define INVALID_ID -1

#define MULTI_SENSOR
//#define DEBUG

PeopleTracker::PeopleTracker() : detect_seq(0), marker_seq(0) {
  ros::NodeHandle n;
  
  listener = new tf::TransformListener();
  startup_time_str = num_to_str<double>(ros::Time::now().toSec());
  
  // Declare variables that can be modified by launch file or command line.
  std::string pub_topic;
  std::string pub_topic_pose_array;
  std::string pub_topic_people;
  std::string pub_topic_trajectory;
  std::string pub_topic_trajectory_acc;
  std::string pub_topic_marker;
  
  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("base_frame", base_frame, std::string("base_link"));
  private_node_handle.param("target_frame", target_frame, std::string("map"));
  private_node_handle.param("tracker_frequency", tracker_frequency, double(30.0));
  private_node_handle.param("log_trajectories", log_trajectories, false);
#ifdef ONLINE_LEARNING
#ifdef MULTI_SENSOR
  private_node_handle.param("human_track_proba", human_track_proba, (double)0.7);
#else
  private_node_handle.param("human_path_min", human_path_min, double(3.0));
  private_node_handle.param("human_velo_min", human_velo_min, double(0.3));
  private_node_handle.param("human_velo_max", human_velo_max, double(1.0));
  private_node_handle.param("human_vari_max", human_vari_max, double(1.0));
  private_node_handle.param("static_path_max", static_path_max, double(0.2));
  private_node_handle.param("static_velo_max", static_velo_max, double(0.2));
  private_node_handle.param("static_vari_max", static_vari_max, double(0.02));
  private_node_handle.param("repeated_poses_max", repeated_poses_max, int(30));
#endif
#endif
  parseParams(private_node_handle);
  
  // Create a status callback.
  ros::SubscriberStatusCallback con_cb = boost::bind(&PeopleTracker::connectCallback, this, boost::ref(n));
  
  private_node_handle.param("positions", pub_topic, std::string("/people_tracker/positions"));
  pub_detect = n.advertise<bayes_people_tracker::PeopleTracker>(pub_topic.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("pose_array", pub_topic_pose_array, std::string("/people_tracker/pose_array"));
  pub_pose_array = n.advertise<geometry_msgs::PoseArray>(pub_topic_pose_array.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("people", pub_topic_people, std::string("/people_tracker/people"));
  pub_people = n.advertise<people_msgs::People>(pub_topic_people.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("trajectory", pub_topic_trajectory, std::string("/people_tracker/trajectory"));
  pub_trajectory = n.advertise<geometry_msgs::PoseArray>(pub_topic_trajectory.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("trajectory_acc", pub_topic_trajectory_acc, std::string("/people_tracker/trajectory_acc"));
  pub_trajectory_acc = n.advertise<people_msgs::People>(pub_topic_trajectory_acc.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("marker", pub_topic_marker, std::string("/people_tracker/marker_array"));
  pub_marker = n.advertise<visualization_msgs::MarkerArray>(pub_topic_marker.c_str(), 100, con_cb, con_cb);
  
  boost::thread tracking_thread(boost::bind(&PeopleTracker::trackingThread, this));
  
  ros::spin();
}

void PeopleTracker::parseParams(ros::NodeHandle n) {
  std::string filter;
  n.getParam("filter_type", filter);
  ROS_INFO("[%s] Found filter type: %s", __APP_NAME__, filter.c_str());
  
  if(filter == "EKF") {
    if(n.hasParam("std_limit")) {
      double stdLimit;
      n.getParam("std_limit", stdLimit);
      ROS_INFO("[%s] std_limit: %f", __APP_NAME__, stdLimit);
      ekf = new SimpleTracking<EKFilter>(stdLimit);
    } else {
      ekf = new SimpleTracking<EKFilter>();
    }
  } else if(filter == "UKF") {
    if(n.hasParam("std_limit")) {
      double stdLimit;
      n.getParam("std_limit", stdLimit);
      ROS_INFO("[%s] std_limit: %f", __APP_NAME__, stdLimit);
      ukf = new SimpleTracking<UKFilter>(stdLimit);
    } else {
      ukf = new SimpleTracking<UKFilter>();
    }
  } else if(filter == "PF") {
    if(n.hasParam("std_limit")) {
      double stdLimit;
      n.getParam("std_limit", stdLimit);
      ROS_INFO("[%s] std_limit: %f", __APP_NAME__, stdLimit);
      pf = new SimpleTracking<PFilter>(stdLimit);
    } else {
      pf = new SimpleTracking<PFilter>();
    }
  } else {
    ROS_FATAL("[%s] Filter type %s is not specified. Unable to create the tracker. Please use either EKF, UKF or PF.", __APP_NAME__, filter.c_str());
    return;
  }
  
  XmlRpc::XmlRpcValue cv_noise;
  n.getParam("cv_noise_params", cv_noise);
  ROS_ASSERT(cv_noise.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  ROS_INFO_STREAM("Constant Velocity Model noise: " << cv_noise);
  if(ekf == NULL) {
    if(ukf == NULL) {
      pf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    } else {
      ukf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    }
  } else {
    ekf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
  }
  ROS_INFO_STREAM("Created " << filter << " based tracker using constant velocity prediction model.");
  
  XmlRpc::XmlRpcValue detectors;
  n.getParam("detectors", detectors);
  ROS_ASSERT(detectors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = detectors.begin(); it != detectors.end(); ++it) {
    ROS_INFO_STREAM("Found detector: " << (std::string)(it->first) << " ==> " << detectors[it->first]);
    try {
      if(ekf == NULL) {
	if(ukf == NULL) {
	  if(detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
	    int seq_size = detectors[it->first]["seq_size"];
	    pf->addDetectorModel(it->first,
				 detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				 detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR: detectors[it->first]["observation_model"] == "BEARING"? BEARING  : throw(observ_exception()),
				 detectors[it->first]["noise_params"]["x"],
				 detectors[it->first]["noise_params"]["y"],(unsigned int) seq_size, detectors[it->first]["seq_time"]);
	  } else {
	    pf->addDetectorModel(it->first,
				 detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				 detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR: detectors[it->first]["observation_model"] == "BEARING"? BEARING  : throw(observ_exception()),
				 detectors[it->first]["noise_params"]["x"],
				 detectors[it->first]["noise_params"]["y"]);
	  }
	} else {
	  if(detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
	    int seq_size = detectors[it->first]["seq_size"];
	    ukf->addDetectorModel(it->first,
				  detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				 detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR: detectors[it->first]["observation_model"] == "BEARING"? BEARING  : throw(observ_exception()),
				  detectors[it->first]["noise_params"]["x"],
				  detectors[it->first]["noise_params"]["y"],(unsigned int) seq_size,detectors[it->first]["seq_time"]);
	  } else {
	    ukf->addDetectorModel(it->first,
				  detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
          detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR: detectors[it->first]["observation_model"] == "BEARING"? BEARING  : throw(observ_exception()),
				  detectors[it->first]["noise_params"]["x"],
				  detectors[it->first]["noise_params"]["y"]);
	  }
	}
      } else {
	if(detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
	  int seq_size = detectors[it->first]["seq_size"];
	  ekf->addDetectorModel(it->first,
				detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
				detectors[it->first]["noise_params"]["x"],
				detectors[it->first]["noise_params"]["y"],(unsigned int) seq_size,detectors[it->first]["seq_time"]);
	} else {
	  ekf->addDetectorModel(it->first,
				detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
				detectors[it->first]["noise_params"]["x"],
				detectors[it->first]["noise_params"]["y"]);
	}
      }
    } catch (asso_exception& e) {
      ROS_FATAL_STREAM(""
		       << e.what()
		       << " "
		       << detectors[it->first]["matching_algorithm"]
		       << " is not specified. Unable to add "
		       << (std::string)(it->first)
		       << " to the tracker. Please use either NN or NNJPDA as association algorithms."
		       );
      return;
    } catch (observ_exception& e) {
      ROS_FATAL_STREAM(""
		       << e.what()
		       << " "
		       << detectors[it->first]["observation_model"]
		       << " is not specified. Unable to add "
		       << (std::string)(it->first)
		       << " to the tracker. Please use either CARTESIAN or POLAR as observation models."
		       );
      return;
    }
    ros::Subscriber sub;
    subscribers[std::pair<std::string, std::string>(it->first, detectors[it->first]["topic"])] = sub;
  }
}

void PeopleTracker::trackingThread() {
  ros::Rate fps(tracker_frequency);
  double time_sec = 0.0;
  
  while(ros::ok()) {
    std::map<long, std::vector<people_msgs::Person> > ppl;
    if(ekf == NULL) {
      if(ukf == NULL) {
	ppl = pf->track(&time_sec);
      } else {
	ppl = ukf->track(&time_sec);
      }
    } else {
      ppl = ekf->track(&time_sec);
    }
    
    if(ppl.size()) {
      std::vector<people_msgs::Person> people;
      std::vector<people_msgs::Person> variances;
      std::vector<std::string> uuids;
      std::vector<long> pids;
      
      for(std::map<long, std::vector<people_msgs::Person> >::const_iterator it = ppl.begin(); it != ppl.end(); ++it) {
	people.push_back(it->second[0]);
	variances.push_back(it->second[1]);
	uuids.push_back(generateUUID(startup_time_str, it->first));
	pids.push_back(it->first);
      }
      
      /*** for STRANDS ***/
      // if(pub_detect.getNumSubscribers() || pub_pose_array.getNumSubscribers() || pub_people.getNumSubscribers()) {
      // 	publishDetections(time_sec, poses, vels, uuids, distances, angles, min_dist, angle);
      // }
      
      if(pub_marker.getNumSubscribers()) {
	createVisualisation(people, pids, pub_marker);
      }
      
      if(pub_trajectory_acc.getNumSubscribers()) {
	people_msgs::People trajectory_acc;
      	trajectory_acc.header.stamp = ros::Time::now();
      	trajectory_acc.header.frame_id = target_frame;
      	trajectory_acc.people = people;
      	pub_trajectory_acc.publish(trajectory_acc);
      }
      
      publishTrajectory(people, variances, pids, pub_trajectory);
    }
    
    fps.sleep();
  }
}

void PeopleTracker::publishDetections(double time_sec,
				      std::vector<geometry_msgs::Pose> ppl,
				      std::vector<geometry_msgs::Pose> vels,
				      std::vector<std::string> uuids,
				      std::vector<double> distances,
				      std::vector<double> angles,
				      double min_dist,
				      double angle) {
  bayes_people_tracker::PeopleTracker result;
  result.header.stamp.fromSec(time_sec);
  result.header.frame_id = target_frame;
  result.header.seq = ++detect_seq;
  result.poses = ppl;
  result.uuids = uuids;
  result.distances = distances;
  result.angles = angles;
  result.min_distance = min_dist;
  result.min_distance_angle = angle;
  publishDetections(result);
  
  geometry_msgs::PoseArray poses;
  poses.header = result.header;
  poses.poses = ppl;
  publishDetections(poses);
  
  people_msgs::People people;
  people.header = result.header;
  for(int i = 0; i < ppl.size(); i++) {
    people_msgs::Person person;
    person.position = ppl[i].position;
    person.velocity = vels[i].position;
    person.name = uuids[i];
    person.tags.push_back(uuids[i]);
    person.tagnames.push_back("uuid");
    person.reliability = 1.0;
    people.people.push_back(person);
  }
  publishDetections(people);
}

void PeopleTracker::publishDetections(bayes_people_tracker::PeopleTracker msg) {
  pub_detect.publish(msg);
}

void PeopleTracker::publishDetections(geometry_msgs::PoseArray msg) {
  pub_pose_array.publish(msg);
}

void PeopleTracker::publishDetections(people_msgs::People msg) {
  pub_people.publish(msg);
}

void PeopleTracker::PN_experts(geometry_msgs::PoseArray &variance, geometry_msgs::PoseArray &velocity, geometry_msgs::PoseArray &trajectory) {
  float path_length = 0.0;
  for(int i = 1; i < trajectory.poses.size(); i++) {
    path_length += hypot(trajectory.poses[i].position.x-trajectory.poses[i-1].position.x, trajectory.poses[i].position.y-trajectory.poses[i-1].position.y);
  }
  float sum_velocity = 0.0;
  for(int i = 0; i < velocity.poses.size(); i++) {
    sum_velocity += fabs(velocity.poses[i].position.x+velocity.poses[i].position.y);
  }
  float avg_velocity = sum_velocity / velocity.poses.size();
  float sum_variance = 0.0;
  for(int i = 0; i < variance.poses.size(); i++) {
    sum_variance += variance.poses[i].position.x+variance.poses[i].position.y;
  }
  float avg_variance = sum_variance / variance.poses.size();
  if(path_length >= human_path_min && avg_velocity >= human_velo_min && avg_velocity <= human_velo_max) {
    trajectory.header.frame_id = "human_trajectory";
  }
  if(path_length <= static_path_max && avg_velocity <= static_velo_max && avg_variance <= static_vari_max) {
    trajectory.header.frame_id = "non_human_trajectory";
  }
  //std::cerr << "path_length = " << path_length << ", avg_velocity = " << avg_velocity << ", avg_variance = " << avg_variance << std::endl;
}

void PeopleTracker::track_probability(geometry_msgs::PoseArray &trajectory) {
  std::vector<double> odds;
  for(int i = 0; i < trajectory.poses.size(); i++) {
    double p = trajectory.poses[i].orientation.x;
    if(p > 0) {
      if(std::fabs(1.0 - p) < DBL_EPSILON) {
	odds.push_back(100.0);
      } else {
	odds.push_back(p / (1.0 - p));
      }
    }
  }
  
  double product_odds = 1.0; // A tracklet's human probability should always start with P(H) = 0.5.
  for(int i = 0; i < odds.size(); i++) {
    product_odds *= odds[i];
  }
  
  double track_probability = (product_odds / (1.0 + product_odds));
  ROS_INFO("[%s] The probability that trajectory ID %d belongs to a human is %f", __APP_NAME__, trajectory.header.seq, track_probability);
  
  if(track_probability > human_track_proba) {
    trajectory.header.frame_id = "human_trajectory";
  }
  if(track_probability < 1.0 - human_track_proba) {
    trajectory.header.frame_id = "non_human_trajectory";
  }
}

void PeopleTracker::publishTrajectory(std::vector<people_msgs::Person> people,
				      std::vector<people_msgs::Person> variances,
				      std::vector<long> pids,
				      ros::Publisher& pub) {
#if defined(ONLINE_LEARNING) && !defined(MULTI_SENSOR)
  /*** find how many repeated poses in previous_poses ***/
  //@TODO fusing with "find trajectories" block
  bool checked[previous_poses.size()];
  for(int i = 0; i < previous_poses.size(); i++)
    checked[i] = false;
  for(int i = 0; i < previous_poses.size(); i++) {
    if(!checked[i] && boost::get<0>(previous_poses[i]) != INVALID_ID) {
      int n = 0;
      for(int j = i; j < previous_poses.size(); j++) {
	if(boost::get<0>(previous_poses[j]) == boost::get<0>(previous_poses[i])) {
	  n++;
	  checked[j] = true;
	}
      }

      bool publish_and_remove = false;
      geometry_msgs::PoseArray trajectory;
      geometry_msgs::PoseArray velocity;
      geometry_msgs::PoseArray variance;
      geometry_msgs::Pose p;
      
      if(n > repeated_poses_max) {
	for(int j = 0; j < previous_poses.size(); j++) {
	  if(boost::get<0>(previous_poses[j]) == boost::get<0>(previous_poses[i])) {
	    p.position = boost::get<1>(previous_poses[j]).position;
	    trajectory.poses.push_back(p);
	    p.position = boost::get<1>(previous_poses[j]).velocity;
	    velocity.poses.push_back(p);
	    p.position = boost::get<2>(previous_poses[j]).position;
	    variance.poses.push_back(p);
	  }
	}
	PN_experts(variance, velocity, trajectory);
	if(trajectory.header.frame_id == "non_human_trajectory") {
	  publish_and_remove = true;
	}
      }
      if(publish_and_remove) { //@TODO n > 999 for memory safety
	//std::cerr << "ID = " << boost::get<0>(previous_poses[i]) << ", repeated " << n << std::endl;
	trajectory.header.seq = boost::get<0>(previous_poses[i]); // tracking ID
	trajectory.header.stamp = ros::Time::now();
	pub.publish(trajectory);
	for(int j = 0; j < previous_poses.size(); j++) {
	  if(boost::get<0>(previous_poses[j]) == trajectory.header.seq) {
	    boost::get<0>(previous_poses[j]) = INVALID_ID;
	  }
	}
      }
    }
  }
#endif
  
  /*** find trajectories ***/ 
  for(int i = 0; i < previous_poses.size(); i++) {
    if(boost::get<0>(previous_poses[i]) != INVALID_ID) {
      bool last_pose = true;
      for(int j = 0; j < pids.size(); j++) {
  	if(pids[j] == boost::get<0>(previous_poses[i])) {
  	  last_pose = false;
  	  break;
  	}
      }
      if(last_pose) {
  	geometry_msgs::PoseArray trajectory;
  	geometry_msgs::PoseArray velocity;
  	geometry_msgs::PoseArray variance;
	geometry_msgs::Pose p;
	
  	trajectory.header.seq = boost::get<0>(previous_poses[i]); // tracking ID
  	trajectory.header.stamp = ros::Time::now();
  	trajectory.header.frame_id = target_frame; // will be replaced by P-N experts
  	for(int j = 0; j < previous_poses.size(); j++) {
  	  if(boost::get<0>(previous_poses[j]) == trajectory.header.seq) {
	    p.position = boost::get<1>(previous_poses[j]).position;
#ifdef ONLINE_LEARNING
	    if(boost::get<1>(previous_poses[j]).name == "object3d_detector") { // or "leg_detector"
	      p.position.z = std::atoi(boost::get<1>(previous_poses[j]).tags[0].c_str());
	    } else {
	      p.position.z = UNKNOWN;
	    }
#endif
#ifdef MULTI_SENSOR
	    p.orientation.x = boost::get<1>(previous_poses[j]).reliability; // For information transfer only
#endif
	    trajectory.poses.push_back(p);
	    p.position = boost::get<1>(previous_poses[j]).velocity;
	    velocity.poses.push_back(p);
	    p.position = boost::get<2>(previous_poses[j]).position;
	    variance.poses.push_back(p);
	    boost::get<0>(previous_poses[j]) = INVALID_ID;
  	  }
  	}
#ifdef ONLINE_LEARNING
#ifdef MULTI_SENSOR
	track_probability(trajectory);
#else
  	PN_experts(variance, velocity, trajectory);
#endif
#endif
  	pub.publish(trajectory);
  	//std::cerr << "[people_tracker] trajectory ID = " << trajectory.header.seq << ", timestamp = " << trajectory.header.stamp << ", poses size = " << trajectory.poses.size() << std::endl;
	if(log_trajectories) {
	  std::cerr << "trajectory: ";
	  for(int k = 0; k < trajectory.poses.size(); k++) {
	    std::cerr << trajectory.poses[k].position.x << " " << trajectory.poses[k].position.y << " ";
	  }
	  std::cerr << std::endl;
	}
      }
    }
  }
  
  /*** clean up ***/
  for(int i = 0; i < previous_poses.size(); i++) {
    if(boost::get<0>(previous_poses[i]) == INVALID_ID) {
      previous_poses.erase(previous_poses.begin()+i);
    }
  }
  
  /*** add new coming poses to the previous_poses list ***/  
  for(int i = 0; i < people.size(); i++) {
    bool new_pose = true;
#ifdef ONLINE_LEARNING
    for(int j = 0; j < previous_poses.size(); j++) {
      if(people[i].tags[0] != std::to_string(UNKNOWN) && boost::get<1>(previous_poses[j]).tags[0] == people[i].tags[0]) {
	new_pose = false;
	break;
      }
    }    
#endif
    if(new_pose) {
      //if(vars[i].position.x+vars[i].position.y <= human_vari_max) // only use this for learning!
      previous_poses.push_back(boost::make_tuple(pids[i], people[i], variances[i]));
    }
  }
}

void PeopleTracker::createVisualisation(std::vector<people_msgs::Person> people, std::vector<long> pids, ros::Publisher& pub) {
  ROS_DEBUG("[%s] Creating markers.", __APP_NAME__);

  visualization_msgs::MarkerArray marker_array;
  
  for(int i = 0; i < people.size(); i++) {
    /*** for STRANDS ***/
    // std::vector<visualization_msgs::Marker> human = createHuman(i*10, poses[i]);
    // marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
    
    /*** for FLOBOT - track ID ***/
    double human_height = 1.7; //meter
    visualization_msgs::Marker track_id;
    track_id.header.stamp = ros::Time::now();
    track_id.header.frame_id = target_frame;
    track_id.ns = "people_id";
    track_id.id = pids[i];
    track_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    track_id.pose.position.x = people[i].position.x;
    track_id.pose.position.y = people[i].position.y;
    track_id.pose.position.z = human_height;
    track_id.scale.z = 0.7;
    track_id.color.a = 1.0;
    track_id.color.r = 1.0;
    track_id.color.g = 0.2;
    track_id.color.b = 0.0;
    track_id.text = boost::to_string(pids[i]);
    track_id.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(track_id);
    
    /*** for FLOBOT - track ***/
    visualization_msgs::Marker track;
    track.header.stamp = ros::Time::now();
    track.header.frame_id = target_frame;
    track.ns = "people_trajectory";
    track.id = pids[i];
    track.type = visualization_msgs::Marker::LINE_STRIP;
    geometry_msgs::Point p;
    for(int j = 0; j < previous_poses.size(); j++) {
      if(boost::get<0>(previous_poses[j]) == pids[i]) {
	p.x = boost::get<1>(previous_poses[j]).position.x;
	p.y = boost::get<1>(previous_poses[j]).position.y;
	track.points.push_back(p);
      }
    }
    track.scale.x = 0.1;
    track.color.a = 1.0;
    track.color.r = std::max(0.3,(double)(pids[i]%3)/3.0);
    track.color.g = std::max(0.3,(double)(pids[i]%6)/6.0);
    track.color.b = std::max(0.3,(double)(pids[i]%9)/9.0);
    track.lifetime = ros::Duration(1.0);
    marker_array.markers.push_back(track);
  }
  
  pub.publish(marker_array);
}

// detector == pma->people[i].name
void PeopleTracker::detectorCallback(const people_msgs::PositionMeasurementArray::ConstPtr& pma, std::string detector) {
  /* DEBUG print */
  // std::cout << "[" << __APP_NAME__ << "] Received [" << pma->people.size() << "] samples from [" << detector << "], and the sample IDs are:";
  // for(size_t i = 0; i < pma->people.size(); i++) {
  //   std::cout << " " << pma->people[i].object_id;  // sample ID
  // }
  // std::cout << std::endl;
  
  // Publish an empty message to trigger callbacks even when there are no detections.
  // This can be used by nodes which might also want to know when there is no human detected.
  if(pma->people.size() == 0) {
    bayes_people_tracker::PeopleTracker empty;
    empty.header.stamp = ros::Time::now();
    empty.header.frame_id = target_frame;
    empty.header.seq = ++detect_seq;
    publishDetections(empty);
    return;
  }
  
  geometry_msgs::PointStamped person_in_cam_coords;
  geometry_msgs::PointStamped person_in_target_coords;
  people_msgs::PositionMeasurementArray people_in_target_coords;
  
  person_in_cam_coords.header = pma->header;
  if(person_in_cam_coords.header.frame_id.empty()) {
    for(int i = 0; i < pma->people.size(); i++) {
      if(!pma->people[i].header.frame_id.empty()) {
	person_in_cam_coords.header.frame_id = pma->people[i].header.frame_id;
	break;
      }
    }
  }
  
  for(int i = 0; i < pma->people.size(); i++) {
    try {
      ROS_DEBUG("[%s] Transforming received position into %s coordinate system.",  __APP_NAME__, target_frame.c_str());
      
      person_in_cam_coords.point = pma->people[i].pos;
      
      listener->waitForTransform(person_in_cam_coords.header.frame_id, target_frame, person_in_cam_coords.header.stamp, ros::Duration(1.0));
      listener->transformPoint(target_frame, ros::Time(0), person_in_cam_coords, person_in_cam_coords.header.frame_id, person_in_target_coords);
      
      people_msgs::PositionMeasurement pm;
      
      if(pma->people[i].name.empty()) {
	pm.name = detector;
      } else {
	pm.name = pma->people[i].name;
      }
      pm.object_id = pma->people[i].object_id;
      pm.pos = person_in_target_coords.point;
      pm.reliability = pma->people[i].reliability;
      people_in_target_coords.people.push_back(pm);
    } catch(tf::TransformException ex) {
      ROS_WARN("[%s] Failed transform: %s", __APP_NAME__, ex.what());
      return;
    }
  }
  
  if(people_in_target_coords.people.size()) {
    if(ekf == NULL) {
      if(ukf == NULL) {
  	pf->addObservation(detector, people_in_target_coords, pma->header.stamp.toSec());
      } else {
  	ukf->addObservation(detector, people_in_target_coords, pma->header.stamp.toSec());
      }
    } else {
      ekf->addObservation(detector, people_in_target_coords, pma->header.stamp.toSec());
    }
  }
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void PeopleTracker::connectCallback(ros::NodeHandle &n) {
  bool loc = pub_detect.getNumSubscribers();
  bool pose_array = pub_pose_array.getNumSubscribers();
  bool people = pub_people.getNumSubscribers();
  bool trajectory = pub_trajectory.getNumSubscribers();
  bool trajectory_acc = pub_trajectory_acc.getNumSubscribers();
  bool markers = pub_marker.getNumSubscribers();
  std::map<std::pair<std::string, std::string>, ros::Subscriber>::const_iterator it;
  
  if(!loc && !pose_array && !people && !trajectory && !trajectory_acc && !markers) {
    ROS_WARN("[%s] No subscribers. Unsubscribing.", __APP_NAME__);
    for(it = subscribers.begin(); it != subscribers.end(); ++it) {
      const_cast<ros::Subscriber&>(it->second).shutdown();
    }
  } else {
    ROS_INFO("[%s] New subscribers. Subscribing.", __APP_NAME__);
    for(it = subscribers.begin(); it != subscribers.end(); ++it) {
      subscribers[it->first] = n.subscribe<people_msgs::PositionMeasurementArray>(it->first.second.c_str(), 100, boost::bind(&PeopleTracker::detectorCallback, this, _1, it->first.first));
    }
  }
}

int main(int argc, char **argv) {
#ifdef ONLINE_LEARNING
  ros::init(argc, argv, "bayes_people_tracker_ol");
#else
  ros::init(argc, argv, "bayes_people_tracker");
#endif
  PeopleTracker* t = new PeopleTracker();
  return 0;
}
