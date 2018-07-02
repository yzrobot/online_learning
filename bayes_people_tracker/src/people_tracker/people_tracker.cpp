#include "people_tracker/people_tracker.h"

//#define MULTI_SENSOR

PeopleTracker::PeopleTracker() : detect_seq(0), marker_seq(0) {
  ros::NodeHandle n;
  
  listener = new tf::TransformListener();
  
  startup_time = ros::Time::now().toSec();
  startup_time_str = num_to_str<double>(startup_time);
  
  // Declare variables that can be modified by launch file or command line.
  std::string pub_topic;
  std::string pub_topic_pose;
  std::string pub_topic_pose_array;
  std::string pub_topic_people;
  std::string pub_topic_trajectory;
  std::string pub_topic_trajectory_acc;
  std::string pub_topic_marker;
  
  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("base_link", base_link, std::string("base_link"));
  private_node_handle.param("target_frame", target_frame, std::string("base_link"));
  private_node_handle.param("tracker_frequency", tracker_frequency, double(30.0));
  private_node_handle.param("log_trajectories", log_trajectories, false);
#if defined(ONLINE_LEARNING) && !defined(MULTI_SENSOR)
  private_node_handle.param("human_path_min", human_path_min, double(3.0));
  private_node_handle.param("human_velo_min", human_velo_min, double(0.3));
  private_node_handle.param("human_velo_max", human_velo_max, double(1.0));
  private_node_handle.param("human_vari_max", human_vari_max, double(1.0));
  private_node_handle.param("static_path_max", static_path_max, double(0.2));
  private_node_handle.param("static_velo_max", static_velo_max, double(0.2));
  private_node_handle.param("static_vari_max", static_vari_max, double(0.02));
  private_node_handle.param("repeated_poses_max", repeated_poses_max, int(30));
#endif
  parseParams(private_node_handle);
  
  // Create a status callback.
  ros::SubscriberStatusCallback con_cb = boost::bind(&PeopleTracker::connectCallback, this, boost::ref(n));
  
  private_node_handle.param("positions", pub_topic, std::string("/people_tracker/positions"));
  pub_detect = n.advertise<bayes_people_tracker::PeopleTracker>(pub_topic.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("pose", pub_topic_pose, std::string("/people_tracker/pose"));
  pub_pose = n.advertise<geometry_msgs::PoseStamped>(pub_topic_pose.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("pose_array", pub_topic_pose_array, std::string("/people_tracker/pose_array"));
  pub_pose_array = n.advertise<geometry_msgs::PoseArray>(pub_topic_pose_array.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("people", pub_topic_people, std::string("/people_tracker/people"));
  pub_people = n.advertise<people_msgs::People>(pub_topic_people.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("trajectory", pub_topic_trajectory, std::string("/people_tracker/trajectory"));
  pub_trajectory = n.advertise<geometry_msgs::PoseArray>(pub_topic_trajectory.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("trajectory_acc", pub_topic_trajectory_acc, std::string("/people_tracker/trajectory_acc"));
  pub_trajectory_acc = n.advertise<geometry_msgs::PoseArray>(pub_topic_trajectory_acc.c_str(), 100, con_cb, con_cb);
  private_node_handle.param("marker", pub_topic_marker, std::string("/people_tracker/marker_array"));
  pub_marker = n.advertise<visualization_msgs::MarkerArray>(pub_topic_marker.c_str(), 100, con_cb, con_cb);
  
  boost::thread tracking_thread(boost::bind(&PeopleTracker::trackingThread, this));
  
  ros::spin();
}

void PeopleTracker::parseParams(ros::NodeHandle n) {
  std::string filter;
  n.getParam("filter_type", filter);
  ROS_INFO_STREAM("Found filter type: " << filter);
  if (filter == "EKF") {
    if (n.hasParam("std_limit")) {
      double stdLimit;
      n.getParam("std_limit", stdLimit);
      ROS_INFO("std_limit: %f ",stdLimit);
      ekf = new SimpleTracking<EKFilter>(stdLimit);
    } else {
      ekf = new SimpleTracking<EKFilter>();
    }
  } else if (filter == "UKF") {
    if (n.hasParam("std_limit")) {
      double stdLimit;
      n.getParam("std_limit", stdLimit);
      ROS_INFO("std_limit: %f ",stdLimit);
      ukf = new SimpleTracking<UKFilter>(stdLimit);
    } else {
      ukf = new SimpleTracking<UKFilter>();
    }
  } else if (filter == "PF") {
    if (n.hasParam("std_limit")) {
      double stdLimit;
      n.getParam("std_limit", stdLimit);
      ROS_INFO("std_limit: %f ",stdLimit);
      pf = new SimpleTracking<PFilter>(stdLimit);
    } else {
      pf = new SimpleTracking<PFilter>();
    }
  } else {
    ROS_FATAL_STREAM("Filter type " << filter << " is not specified. Unable to create the tracker. Please use either EKF, UKF or PF.");
    return;
  }
  
  XmlRpc::XmlRpcValue cv_noise;
  n.getParam("cv_noise_params", cv_noise);
  ROS_ASSERT(cv_noise.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  ROS_INFO_STREAM("Constant Velocity Model noise: " << cv_noise);
  if (ekf == NULL) {
    if (ukf == NULL) {
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
      if (ekf == NULL) {
	if (ukf == NULL) {
	  if (detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
	    int seq_size = detectors[it->first]["seq_size"];
	    pf->addDetectorModel(it->first,
				 detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				 detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
				 detectors[it->first]["noise_params"]["x"],
				 detectors[it->first]["noise_params"]["y"],(unsigned int) seq_size, detectors[it->first]["seq_time"]);
	  } else {
	    pf->addDetectorModel(it->first,
				 detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				 detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
				 detectors[it->first]["noise_params"]["x"],
				 detectors[it->first]["noise_params"]["y"]);
	  }
	} else {
	  if (detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
	    int seq_size = detectors[it->first]["seq_size"];
	    ukf->addDetectorModel(it->first,
				  detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				  detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
				  detectors[it->first]["noise_params"]["x"],
				  detectors[it->first]["noise_params"]["y"],(unsigned int) seq_size,detectors[it->first]["seq_time"]);
	  } else {
	    ukf->addDetectorModel(it->first,
				  detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
				  detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
				  detectors[it->first]["noise_params"]["x"],
				  detectors[it->first]["noise_params"]["y"]);
	  }
	}
      } else {
	if (detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
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
    std::map<long, std::vector<geometry_msgs::Pose> > ppl;
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
      geometry_msgs::Pose closest_person_point;
      std::vector<geometry_msgs::Pose> poses;
      std::vector<geometry_msgs::Pose> vels;
      std::vector<geometry_msgs::Pose> vars;
      std::vector<std::string> uuids;
      std::vector<long> pids;
      std::vector<double> distances;
      std::vector<double> angles;
      double min_dist = DBL_MAX;
      double angle;
      
      for(std::map<long, std::vector<geometry_msgs::Pose> >::const_iterator it = ppl.begin(); it != ppl.end(); ++it) {
	poses.push_back(it->second[0]);
	vels.push_back(it->second[1]);
	vars.push_back(it->second[2]);
	uuids.push_back(generateUUID(startup_time_str, it->first));
	pids.push_back(it->first);
	
	geometry_msgs::PoseStamped poseInRobotCoords;
	geometry_msgs::PoseStamped poseInTargetCoords;
	poseInTargetCoords.header.frame_id = target_frame;
	poseInTargetCoords.header.stamp.fromSec(time_sec);
	poseInTargetCoords.pose = it->second[0];
	
	//Find closest person and get distance and angle
	if(strcmp(target_frame.c_str(), base_link.c_str())) {
	  try {
	    ROS_DEBUG("Transforming received position into %s coordinate system.", base_link.c_str());
	    listener->waitForTransform(poseInTargetCoords.header.frame_id, base_link, poseInTargetCoords.header.stamp, ros::Duration(1.0));
	    listener->transformPose(base_link, ros::Time(0), poseInTargetCoords, poseInTargetCoords.header.frame_id, poseInRobotCoords);
	  } catch(tf::TransformException ex) {
	    ROS_WARN("Failed transform: %s", ex.what());
	    continue;
	  }
	} else {
	  poseInRobotCoords = poseInTargetCoords;
	}
	
	if(pub_detect.getNumSubscribers() || pub_pose.getNumSubscribers() || pub_pose_array.getNumSubscribers() || pub_people.getNumSubscribers()) {
	  std::vector<double> polar = cartesianToPolar(poseInRobotCoords.pose.position);
	  distances.push_back(polar[0]);
	  angles.push_back(polar[1]);
	  angle = polar[0] < min_dist ? polar[1] : angle;
	  closest_person_point = polar[0] < min_dist ? it->second[0] : closest_person_point;
	  min_dist = polar[0] < min_dist ? polar[0] : min_dist;
	}
      }
      
      if(pub_detect.getNumSubscribers() || pub_pose.getNumSubscribers() || pub_pose_array.getNumSubscribers() || pub_people.getNumSubscribers())
	publishDetections(time_sec, closest_person_point, poses, vels, uuids, distances, angles, min_dist, angle);
      
      if(pub_marker.getNumSubscribers())
	createVisualisation(poses, pids, pub_marker);
      
      if(pub_trajectory_acc.getNumSubscribers()) {
	geometry_msgs::PoseArray trajectory_acc;
	trajectory_acc.header.stamp = ros::Time::now();
  	trajectory_acc.header.frame_id = target_frame;
	trajectory_acc.poses = poses;
	pub_trajectory_acc.publish(trajectory_acc);
      }
      
      //if(pub_trajectory.getNumSubscribers())
      publishTrajectory(poses, vels, vars, pids, pub_trajectory);
    }
    fps.sleep();
  }
}

void PeopleTracker::publishDetections(double time_sec,
				      geometry_msgs::Pose closest,
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
  
  geometry_msgs::PoseStamped pose;
  pose.header = result.header;
  pose.pose = closest;
  publishDetections(pose);
  
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

void PeopleTracker::publishDetections(geometry_msgs::PoseStamped msg) {
  pub_pose.publish(msg);
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
    path_length += hypot(trajectory.poses[i].position.x-trajectory.poses[i-1].position.x,
			 trajectory.poses[i].position.y-trajectory.poses[i-1].position.y);
  }
  float sum_velocity = 0.0;
  for(int i = 0; i < velocity.poses.size(); i++)
    sum_velocity += fabs(velocity.poses[i].position.x+velocity.poses[i].position.y);
  float avg_velocity = sum_velocity / velocity.poses.size();
  float sum_variance = 0.0;
  for(int i = 0; i < variance.poses.size(); i++)
    sum_variance += variance.poses[i].position.x+variance.poses[i].position.y;
  float avg_variance = sum_variance / variance.poses.size();
  //std::cerr << "path_length = " << path_length << ", avg_velocity = " << avg_velocity << ", avg_variance = " << avg_variance << std::endl;
  if(path_length >= human_path_min && avg_velocity >= human_velo_min && avg_velocity <= human_velo_max)
    trajectory.header.frame_id = "human_trajectory";
  if(path_length <= static_path_max && avg_velocity <= static_velo_max && avg_variance <= static_vari_max)
    trajectory.header.frame_id = "static_trajectory";
}

void PeopleTracker::publishTrajectory(std::vector<geometry_msgs::Pose> poses,
				      std::vector<geometry_msgs::Pose> vels,
				      std::vector<geometry_msgs::Pose> vars,
				      std::vector<long> pids,
				      ros::Publisher& pub) {
#if defined(ONLINE_LEARNING) && !defined(MULTI_SENSOR)
  /*** find how many repeated poses in previous_poses ***/
  //@todo really necessary? If so, fusing with "find trajectories"
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
      if(n > repeated_poses_max) {
	for(int j = 0; j < previous_poses.size(); j++) {
	  if(boost::get<0>(previous_poses[j]) == boost::get<0>(previous_poses[i])) {
	    trajectory.poses.push_back(boost::get<3>(previous_poses[j]));
	    velocity.poses.push_back(boost::get<2>(previous_poses[j]));
	    variance.poses.push_back(boost::get<1>(previous_poses[j]));
	  }
	}
	PN_experts(variance, velocity, trajectory);
	if(trajectory.header.frame_id == "static_trajectory")
	  publish_and_remove = true;
      }
      if(publish_and_remove) { //@todo or n > 999 for memory safety
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
  	trajectory.header.seq = boost::get<0>(previous_poses[i]); // tracking ID
  	trajectory.header.stamp = ros::Time::now();
  	trajectory.header.frame_id = target_frame; // will be reused by P-N experts
  	for(int j = 0; j < previous_poses.size(); j++) {
  	  if(boost::get<0>(previous_poses[j]) == trajectory.header.seq) {
  	    trajectory.poses.push_back(boost::get<3>(previous_poses[j]));
  	    velocity.poses.push_back(boost::get<2>(previous_poses[j]));
  	    variance.poses.push_back(boost::get<1>(previous_poses[j]));
  	    boost::get<0>(previous_poses[j]) = INVALID_ID;
  	  }
  	}
#if defined(ONLINE_LEARNING) && !defined(MULTI_SENSOR)
  	PN_experts(variance, velocity, trajectory);
#endif
	//trajectory.poses.insert(trajectory.poses.end(), variance.poses.begin(), variance.poses.end()); //@NB_test
  	pub.publish(trajectory);
  	//std::cerr << "[people_tracker] trajectory ID = " << trajectory.header.seq << ", timestamp = " << trajectory.header.stamp << ", poses size = " << trajectory.poses.size() << std::endl;
	if(log_trajectories) {
	  for(int k = 0; k < trajectory.poses.size(); k++) {
	    std::cerr << trajectory.poses[k].position.x << " " << std::cerr << trajectory.poses[k].position.y << std::endl;
	  }
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
  for(int i = 0; i < poses.size(); i++) {
    bool new_pose = true;
#ifdef ONLINE_LEARNING
    for(int j = 0; j < previous_poses.size(); j++) {
      if(poses[i].position.z >= 0.0 && boost::get<3>(previous_poses[j]).position.z == poses[i].position.z) {
	new_pose = false;
	break;
      }
    }
#endif
    if(new_pose) {
      //if(vars[i].position.x+vars[i].position.y <= human_vari_max) // only use this for learning!
      previous_poses.push_back(boost::make_tuple(pids[i], vars[i], vels[i], poses[i]));
    }
  }
}

void PeopleTracker::createVisualisation(std::vector<geometry_msgs::Pose> poses,
					std::vector<long> pids,
					ros::Publisher& pub) {
  ROS_DEBUG("Creating markers");
  visualization_msgs::MarkerArray marker_array;
  for(int i = 0; i < poses.size(); i++) {
    /* for STRANDS
     * std::vector<visualization_msgs::Marker> human = createHuman(i*10, poses[i]);
     * marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
     */
    /* for FLOBOT - tracking ID */
    double human_height = 1.7; //meter
    visualization_msgs::Marker tracking_id;
    tracking_id.header.stamp = ros::Time::now();
    tracking_id.header.frame_id = target_frame;
    tracking_id.ns = "people_id";
    tracking_id.id = pids[i];
    tracking_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tracking_id.pose.position.x = poses[i].position.x;
    tracking_id.pose.position.y = poses[i].position.y;
    tracking_id.pose.position.z = human_height;
    tracking_id.scale.z = 0.7;
    tracking_id.color.a = 1.0;
    tracking_id.color.r = 1.0;
    tracking_id.color.g = 0.2;
    tracking_id.color.b = 0.0;
    tracking_id.text = boost::to_string(pids[i]);
    tracking_id.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(tracking_id);
    
    /* for FLOBOT - tracking trajectory */
    visualization_msgs::Marker tracking_tr;
    tracking_tr.header.stamp = ros::Time::now();
    tracking_tr.header.frame_id = target_frame;
    tracking_tr.ns = "people_trajectory";
    tracking_tr.id = pids[i];
    tracking_tr.type = visualization_msgs::Marker::LINE_STRIP;
    geometry_msgs::Point p;
    for(int j = 0; j < previous_poses.size(); j++) {
      if(boost::get<0>(previous_poses[j]) == pids[i]) {
	p.x = boost::get<3>(previous_poses[j]).position.x;
	p.y = boost::get<3>(previous_poses[j]).position.y;
	tracking_tr.points.push_back(p);
      }
    }
    tracking_tr.scale.x = 0.1;
    tracking_tr.color.a = 1.0;
    tracking_tr.color.r = std::max(0.3,(double)(pids[i]%3)/3.0);
    tracking_tr.color.g = std::max(0.3,(double)(pids[i]%6)/6.0);
    tracking_tr.color.b = std::max(0.3,(double)(pids[i]%9)/9.0);
    tracking_tr.lifetime = ros::Duration(1.0);
    marker_array.markers.push_back(tracking_tr);
  }
  pub.publish(marker_array);
}

std::vector<double> PeopleTracker::cartesianToPolar(geometry_msgs::Point point) {
  ROS_DEBUG("cartesianToPolar: Cartesian point: x: %f, y: %f, z %f", point.x, point.y, point.z);
  std::vector<double> output;
  double dist = sqrt(pow(point.x,2) + pow(point.y,2));
  double angle = atan2(point.y, point.x);
  output.push_back(dist);
  output.push_back(angle);
  ROS_DEBUG("cartesianToPolar: Polar point: distance: %f, angle: %f", dist, angle);
  return output;
}

void PeopleTracker::detectorCallback(const geometry_msgs::PoseArray::ConstPtr &pta, std::string detector) {
  //std::cerr << "[people_tacker] got " << pta->poses.size() << " poses, from " << detector << std::endl;
  
#ifdef ONLINE_LEARNING
  double tmp[pta->poses.size()];
  for(int i = 0; i < pta->poses.size(); ++i)
    tmp[i] = pta->poses[i].position.z;
#endif
  
  // Publish an empty message to trigger callbacks even when there are no detections.
  // This can be used by nodes which might also want to know when there is no human detected.
  if(pta->poses.size() == 0) {
    bayes_people_tracker::PeopleTracker empty;
    empty.header.stamp = ros::Time::now();
    empty.header.frame_id = target_frame;
    empty.header.seq = ++detect_seq;
    publishDetections(empty);
    return;
  }
  
  geometry_msgs::Pose robotPoseInTargetCoords;
  
  std::vector<geometry_msgs::Point> ppl;
  for(int i = 0; i < pta->poses.size(); i++) {
    geometry_msgs::Pose pt = pta->poses[i];
    
    //Create stamped pose for tf
    geometry_msgs::PoseStamped poseInCamCoords;
    geometry_msgs::PoseStamped poseInTargetCoords;
    poseInCamCoords.header = pta->header;
    poseInCamCoords.pose = pt;
    
    //Transform
    try {
      // Transform into given traget frame. Default /map
      ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
      listener->waitForTransform(poseInCamCoords.header.frame_id, target_frame, poseInCamCoords.header.stamp, ros::Duration(1.0));
      listener->transformPose(target_frame, ros::Time(0), poseInCamCoords, poseInCamCoords.header.frame_id, poseInTargetCoords);
      
      // @todo use http://wiki.ros.org/pose_publisher
      tf::StampedTransform transform;
      listener->lookupTransform(target_frame, base_link, ros::Time(0), transform);
      robotPoseInTargetCoords.position.x = transform.getOrigin().getX();
      robotPoseInTargetCoords.position.y = transform.getOrigin().getY();
      robotPoseInTargetCoords.position.z = transform.getOrigin().getZ();
      robotPoseInTargetCoords.orientation.x = transform.getRotation().getX();
      robotPoseInTargetCoords.orientation.y = transform.getRotation().getY();
      robotPoseInTargetCoords.orientation.z = transform.getRotation().getZ();
      robotPoseInTargetCoords.orientation.w = transform.getRotation().getW();
      //std::cerr << robotPoseInTargetCoords << std::endl;
    }
    catch(tf::TransformException ex) {
      ROS_WARN("Failed transform: %s", ex.what());
      return;
    }
    
    //poseInTargetCoords.pose.position.z = 0.0; //@todo check if it can be removed.
    ppl.push_back(poseInTargetCoords.pose.position);
  }
  
#ifdef ONLINE_LEARNING
  for(int i = 0; i < ppl.size(); ++i)
    ppl[i].z = tmp[i];
#endif
  
  if(ppl.size()) {
    if(ekf == NULL) {
      if(ukf == NULL) {
	pf->addObservation(detector, ppl, pta->header.stamp.toSec(), robotPoseInTargetCoords);
      } else {
	ukf->addObservation(detector, ppl, pta->header.stamp.toSec(), robotPoseInTargetCoords);
      }
    } else {
      ekf->addObservation(detector, ppl, pta->header.stamp.toSec(), robotPoseInTargetCoords);
    }
  }
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void PeopleTracker::connectCallback(ros::NodeHandle &n) {
  bool loc = pub_detect.getNumSubscribers();
  bool pose = pub_pose.getNumSubscribers();
  bool pose_array = pub_pose_array.getNumSubscribers();
  bool people = pub_people.getNumSubscribers();
  bool trajectory = pub_trajectory.getNumSubscribers();
  bool trajectory_acc = pub_trajectory_acc.getNumSubscribers();
  bool markers = pub_marker.getNumSubscribers();
  std::map<std::pair<std::string, std::string>, ros::Subscriber>::const_iterator it;
  
  if(!loc && !pose && !pose_array && !people && !trajectory && !trajectory_acc && !markers) {
    ROS_DEBUG("Pedestrian Localisation: No subscribers. Unsubscribing.");
    for(it = subscribers.begin(); it != subscribers.end(); ++it)
      const_cast<ros::Subscriber&>(it->second).shutdown();
  } else {
    ROS_DEBUG("Pedestrian Localisation: New subscribers. Subscribing.");
    for(it = subscribers.begin(); it != subscribers.end(); ++it)
      subscribers[it->first] = n.subscribe<geometry_msgs::PoseArray>(it->first.second.c_str(), 1000, boost::bind(&PeopleTracker::detectorCallback, this, _1, it->first.first));
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
