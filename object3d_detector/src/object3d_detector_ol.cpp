// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
// SVM
#include "svm.h"

#define MULTI_SENSOR

typedef struct feature {
  /*** for visualization ***/
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  /*** for classification ***/
  int number_points;
  float min_distance;
  Eigen::Matrix3f covariance_3d;
  Eigen::Matrix3f moment_3d;
  // float partial_covariance_2d[9];
  // float histogram_main_2d[98];
  // float histogram_second_2d[45];
  float slice[20];
  float intensity[27];
} Feature;

static const int FEATURE_SIZE = 61;
float svm_range_[FEATURE_SIZE][2];
float svm_xlower_ = -1.0, svm_xupper_ = 1.0;

class Object3dDetector {
private:
  /*** Publishers and Subscribers ***/
  ros::NodeHandle node_handle_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Publisher  pose_array_pub_;
  ros::Publisher  marker_array_pub_;
  ros::Publisher  filtered_cloud_pub_;
  ros::Publisher  learned_cloud_pub_;
  
  /*** Parameters ***/
  bool print_fps_;
  std::string frame_id_;
  float z_limit_min_;
  float z_limit_max_;
  int cluster_size_min_;
  int cluster_size_max_;
  float vfilter_min_x_;
  float vfilter_max_x_;
  float vfilter_min_y_;
  float vfilter_max_y_;
  float vfilter_min_z_;
  float vfilter_max_z_;
  float cluster_min_z_;
  
  /*** SVM ***/
  std::vector<Feature> features_;
  struct svm_node *svm_node_;
  struct svm_model *svm_model_;
  struct svm_problem svm_problem_;
  struct svm_parameter svm_parameter_;
  float human_probability_;
  int svm_node_size_;
  
  /*** Online learning ***/
  int init_positives_;
  int init_negatives_;
  int round_positives_;
  int round_negatives_;
  int max_positives_;
  int max_negatives_;
  int max_trains_;
  int train_round_;
  int positive_;
  int negative_;
  uint32_t learnable_cluster_id_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > learnable_clusters_;
  std::vector<double> clusters_probability_;
  bool find_the_best_training_parameters_;
  float track_probability_;
  
public:
  Object3dDetector();
  ~Object3dDetector();
  
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);
  void trajectoryCallback(const geometry_msgs::PoseArray::ConstPtr& trajectory);

  void extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
  void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f);
  void saveFeature(Feature &f, struct svm_node *x);
  void classify();
  void train();
};

Object3dDetector::Object3dDetector() {
  /*** Publishers ***/
  ros::NodeHandle private_nh("~");
  pose_array_pub_     = private_nh.advertise<geometry_msgs::PoseArray>("poses", 1);
  marker_array_pub_   = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 1);
  filtered_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  learned_cloud_pub_  = private_nh.advertise<sensor_msgs::PointCloud2>("learned_cloud", 1);
  
  /*** Parameters ***/
  private_nh.param<bool>("print_fps", print_fps_, false);
  private_nh.param<std::string>("frame_id", frame_id_, "velodyne");
  private_nh.param<float>("z_limit_min", z_limit_min_, -0.8);
  private_nh.param<float>("z_limit_max", z_limit_max_, 1.2);
  private_nh.param<int>("cluster_size_min", cluster_size_min_, 5);
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 30000);
  private_nh.param<float>("human_probability", human_probability_, 0.5);
  private_nh.param<int>("init_positives", init_positives_, 5);
  private_nh.param<int>("init_negatives", init_negatives_, 5);
  private_nh.param<int>("round_positives", round_positives_, 100);
  private_nh.param<int>("round_negatives", round_negatives_, 100);
  private_nh.param<int>("max_trains", max_trains_, 11);
  private_nh.param<bool>("find_the_best_training_parameters", find_the_best_training_parameters_, true);
  private_nh.param<float>("track_probability", track_probability_, 0.7);
  
  private_nh.param<float>("vfilter_min_x", vfilter_min_x_, 0.2);
  private_nh.param<float>("vfilter_max_x", vfilter_max_x_, 1.0);
  private_nh.param<float>("vfilter_min_y", vfilter_min_y_, 0.2);
  private_nh.param<float>("vfilter_max_y", vfilter_max_y_, 1.0);
  private_nh.param<float>("vfilter_min_z", vfilter_min_z_, 0.2);
  private_nh.param<float>("vfilter_max_z", vfilter_max_z_, 2.0);
  private_nh.param<float>("cluster_min_z", cluster_min_z_, 2.0);
  
  /*** SVM ***/
  svm_node_ = (struct svm_node *)malloc((FEATURE_SIZE+1)*sizeof(struct svm_node)); // 1 more size for end index (-1)
  
  svm_parameter_.svm_type = C_SVC; // default C_SVC
  svm_parameter_.kernel_type = RBF; // default RBF
  svm_parameter_.degree = 3; // default 3
  svm_parameter_.gamma = 0.02; // default 1.0/(float)FEATURE_SIZE
  svm_parameter_.coef0 = 0; // default 0
  svm_parameter_.cache_size = 256; // default 100
  svm_parameter_.eps = 0.001; // default 0.001
  svm_parameter_.C = 8; // default 1
  svm_parameter_.nr_weight = 0;
  svm_parameter_.weight_label = NULL;
  svm_parameter_.weight = NULL;
  svm_parameter_.nu = 0.5;
  svm_parameter_.p = 0.1;
  svm_parameter_.shrinking = 0;
  svm_parameter_.probability = 1;
  
  svm_node_size_ = (round_positives_+round_negatives_)*(max_trains_-1)+init_positives_+init_negatives_;
  svm_problem_.l = 0;
  svm_problem_.y = (double *)malloc(svm_node_size_*sizeof(double));
  svm_problem_.x = (struct svm_node **)malloc(svm_node_size_*sizeof(struct svm_node *));
  for(int i = 0; i < svm_node_size_; i++) {
    svm_problem_.x[i] = (struct svm_node *)malloc((FEATURE_SIZE + 1)*sizeof(struct svm_node));
  }
  // one-line malloc
  //svm_problem_.x = (struct svm_node **)malloc((max_positives_+max_negatives_)*sizeof(struct svm_node *)+((max_positives_+max_negatives_)*((FEATURE_SIZE+1)*sizeof(struct svm_node))));
  
  /*** Online learning ***/
  max_positives_ = init_positives_;
  max_negatives_ = init_negatives_;
  train_round_ = 0;
  positive_ = 0;
  negative_ = 0;
  learnable_cluster_id_ = 0;
  
  /*** Subscribers ***/
  point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 100, &Object3dDetector::pointCloudCallback, this);
  trajectory_sub_ = node_handle_.subscribe<geometry_msgs::PoseArray>("people_tracker/trajectory", 100, &Object3dDetector::trajectoryCallback, this);
}

Object3dDetector::~Object3dDetector() {
  svm_free_and_destroy_model(&svm_model_);
  free(svm_node_);
  svm_destroy_param(&svm_parameter_);
  free(svm_problem_.y);
  for(int i = 0; i < svm_node_size_; i++) 
    free(svm_problem_.x[i]);
  free(svm_problem_.x);
}

/*** learning ***/
void Object3dDetector::trajectoryCallback(const geometry_msgs::PoseArray::ConstPtr& trajectory) {
  if(train_round_ == max_trains_ || (positive_+negative_) == (max_positives_+max_negatives_)) {
    return;
  }
  
  pcl::PointCloud<pcl::PointXYZI> learned_cloud; // debug publishing
  
#ifdef MULTI_SENSOR
  /*** track probability, leg_reliability_limit = 0.7, evaluation_greedy_NMS_threshold = 0.5 ***/
  bool human_trajectory = false;
  for(int i = 0; i < trajectory->poses.size(); i++) {
    if(trajectory->poses[i].position.z < 0.0) {
      if(train_round_ > 0) {
	std::vector<double> probabilities, odds;
	for(int j = 0; j < trajectory->poses.size(); j++) {
	  for(int k = 0; k < learnable_clusters_.size(); k++) {
	    if((uint32_t)trajectory->poses[j].position.z == learnable_clusters_[k]->header.seq) {
	      probabilities.push_back(clusters_probability_[k]);
	    }
	  }
	}
	if(probabilities.size() > 0) {
	  double product_odds = 1.0;
	  for(int k = 0; k < probabilities.size(); k++) {
	    if(probabilities[k] == 1.0) {
	      odds.push_back(100.0);
	    } else {
	      odds.push_back(probabilities[k] / (1 - probabilities[k]));
	    }
	  }
	  for(int k = 0; k < odds.size(); k++) {
	    product_odds *= odds[k];
	  }
	  //std::cerr << (product_odds / (1 + product_odds)) << std::endl;
	  if(product_odds / (1 + product_odds) > track_probability_) {
	    human_trajectory = true;
	  }
	}
      } else {
	human_trajectory = true;
      }
      break;
    }
  }
  
  if(human_trajectory) {
    bool stop = false;
    for(int i = 0; i < trajectory->poses.size(); i++) {
      for(int j = 0; j < learnable_clusters_.size(); j++) {
  	if((uint32_t)trajectory->poses[i].position.z == learnable_clusters_[j]->header.seq) {
  	  Feature f;
  	  extractFeature(learnable_clusters_[j], f);
  	  saveFeature(f, svm_problem_.x[svm_problem_.l]);
  	  if(positive_ < max_positives_) {
  	    svm_problem_.y[svm_problem_.l++] = 1; // positive label
  	    ++positive_ >= max_positives_ ? stop = true : stop = false;
  	  }
  	  //learned_cloud += *learnable_clusters_[j];
  	  break;
  	}
      }
      if(stop) {
	break;
      }
    }
  }
#else
  bool learn_it = true;
  
  if(train_round_ > 0) {
    learn_it = false;
    for(int i = 0; i < trajectory->poses.size(); i++) {
      for(int j = 0; j < learnable_clusters_.size(); j++) {
	if((uint32_t)trajectory->poses[i].position.z == learnable_clusters_[j]->header.seq && learnable_clusters_[j]->header.frame_id == "human") {
	  learn_it = true;
	  break;
	}
      }
      if(learn_it) break;
    }
  }
  
  if(learn_it) {
    bool stop = false;
    for(int i = 0; i < trajectory->poses.size(); i++) {
      for(int j = 0; j < learnable_clusters_.size(); j++) {
	if((uint32_t)trajectory->poses[i].position.z == learnable_clusters_[j]->header.seq) {
	  Feature f;
	  extractFeature(learnable_clusters_[j], f);
	  saveFeature(f, svm_problem_.x[svm_problem_.l]);
	  if(trajectory->header.frame_id == "human_trajectory" && positive_ < max_positives_) {
	    svm_problem_.y[svm_problem_.l++] = 1; // 1, the positive label
	    ++positive_ >= max_positives_ ? stop = true : stop = false;
	    //learned_cloud += *learnable_clusters_[j];
	  }
	  if(train_round_ > 0 && trajectory->header.frame_id == "static_trajectory" && negative_ < max_negatives_) {
	    svm_problem_.y[svm_problem_.l++] = -1; // negative label
	    ++negative_ >= max_negatives_ ? stop = true : stop = false;
	    //learned_cloud += *learnable_clusters_[j];
	  }
	  break;
	}
      }
      if(stop) break;
    }
  }
#endif
  
  if(learned_cloud_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(learned_cloud, ros_cloud);
    ros_cloud.header.frame_id = frame_id_;
    learned_cloud_pub_.publish(ros_cloud);
  }
}

int frames; clock_t start_time; bool reset = true;//fps
void Object3dDetector::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2) {
  if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc2, *pcl_pc);
  
  extractCluster(pcl_pc);
  classify();
  train();
  
  if(print_fps_)if(++frames>10){std::cerr<<"[object3d_detector_ol]: fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<", positive = "<<positive_<<", negative = "<<negative_<<std::endl;reset = true;}//fps
}

const int nested_regions_ = 14;
int zone_[nested_regions_] = {2,3,3,3,3,3,3,2,3,3,3,3,3,3}; // for more details, see our IROS'17 paper.
void Object3dDetector::extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
  features_.clear();
  
  pcl::IndicesPtr pc_indices(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(pc);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_limit_min_, z_limit_max_);
  pass.filter(*pc_indices);
  
  // if(filtered_cloud_pub_.getNumSubscribers() > 0) {
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZI>);
  //   sensor_msgs::PointCloud2 ros_pc2;
  //   pcl::copyPointCloud(*pc, *pc_indices, *pcl_pc);
  //   pcl::toROSMsg(*pcl_pc, ros_pc2);
  //   filtered_cloud_pub_.publish(ros_pc2);
  // }
  
  boost::array<std::vector<int>, nested_regions_> indices_array;
  for(int i = 0; i < pc_indices->size(); i++) {
    float range = 0.0;
    for(int j = 0; j < nested_regions_; j++) {
      float d2 = pc->points[(*pc_indices)[i]].x * pc->points[(*pc_indices)[i]].x +
	pc->points[(*pc_indices)[i]].y * pc->points[(*pc_indices)[i]].y +
	pc->points[(*pc_indices)[i]].z * pc->points[(*pc_indices)[i]].z;
      if(d2 > range*range && d2 <= (range+zone_[j])*(range+zone_[j])) {
	indices_array[j].push_back((*pc_indices)[i]);
	break;
      }
      range += zone_[j];
    }
  }
  
  float tolerance = 0.0;
  for(int i = 0; i < nested_regions_; i++) {
    tolerance += 0.1;
    if(indices_array[i].size() > cluster_size_min_) {
      boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(pc, indices_array_ptr);
      
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pc);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);
      
      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	  cluster->points.push_back(pc->points[*pit]);
	cluster->width = cluster->size();
	cluster->height = 1;
	cluster->is_dense = true;
	
	Eigen::Vector4f min, max;
	pcl::getMinMax3D(*cluster, min, max);
	if(max[0]-min[0] >= vfilter_min_x_ && max[0]-min[0] <= vfilter_max_x_ &&
	   max[1]-min[1] >= vfilter_min_y_ && max[1]-min[1] <= vfilter_max_y_ &&
	   max[2]-min[2] >= vfilter_min_z_ && max[2]-min[2] <= vfilter_max_z_ &&
	   min[2] <= cluster_min_z_) {
	  Feature f;
	  extractFeature(cluster, f);
	  features_.push_back(f);
	  cluster->header.seq = learnable_cluster_id_++;
	  learnable_clusters_.push_back(cluster);
	} else {
	  if(positive_ == max_positives_ && negative_ < max_negatives_) { // @todo condition test only, to be removed
	    Feature f;
	    extractFeature(cluster, f);
	    saveFeature(f, svm_problem_.x[svm_problem_.l]);
	    svm_problem_.y[svm_problem_.l++] = -1; // negative label
	    ++negative_;
	  }
	}
      }
    }
  }
}

/* *** Feature Extraction ***
 * f1 (1d): the number of points included in a cluster.
 * f2 (1d): the minimum distance of the cluster to the sensor.
 * => f1 and f2 should be used in pairs, since f1 varies with f2 changes.
 * f3 (6d): 3D covariance matrix of the cluster.
 * f4 (6d): the normalized moment of inertia tensor.
 * => Since both f3 and f4 are symmetric, we only use 6 elements from each as features.
 * f5 (9d): 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
 * f6 (98d): The normalized 2D histogram for the main plane, 14 × 7 bins.
 * f7 (45d): The normalized 2D histogram for the secondary plane, 9 × 5 bins.
 * f8 (20d): Slice feature for the cluster.
 * f9 (27d): Intensity.
 */

void computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZI> &pc, Eigen::Matrix3f &moment_3d) {
  moment_3d.setZero();
  for(size_t i = 0; i < pc.size(); i++) {
    moment_3d(0,0) += pc[i].y*pc[i].y+pc[i].z*pc[i].z;
    moment_3d(0,1) -= pc[i].x*pc[i].y;
    moment_3d(0,2) -= pc[i].x*pc[i].z;
    moment_3d(1,1) += pc[i].x*pc[i].x+pc[i].z*pc[i].z;
    moment_3d(1,2) -= pc[i].y*pc[i].z;
    moment_3d(2,2) += pc[i].x*pc[i].x+pc[i].y*pc[i].y;
  }
  moment_3d(1, 0) = moment_3d(0, 1);
  moment_3d(2, 0) = moment_3d(0, 2);
  moment_3d(2, 1) = moment_3d(1, 2);
}

/* Main plane is formed from the maximum and middle eigenvectors.
 * Secondary plane is formed from the middle and minimum eigenvectors.
 */
void computeProjectedPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Eigen::Matrix3f &eigenvectors, int axe, Eigen::Vector4f &centroid, pcl::PointCloud<pcl::PointXYZI>::Ptr plane) {
  Eigen::Vector4f coefficients;
  coefficients[0] = eigenvectors(0,axe);
  coefficients[1] = eigenvectors(1,axe);
  coefficients[2] = eigenvectors(2,axe);
  coefficients[3] = 0;
  coefficients[3] = -1 * coefficients.dot(centroid);
  for(size_t i = 0; i < pc->size(); i++) {
    float distance_to_plane =
      coefficients[0] * pc->points[i].x +
      coefficients[1] * pc->points[i].y +
      coefficients[2] * pc->points[i].z +
      coefficients[3];
    pcl::PointXYZI p;
    p.x = pc->points[i].x - distance_to_plane * coefficients[0];
    p.y = pc->points[i].y - distance_to_plane * coefficients[1];
    p.z = pc->points[i].z - distance_to_plane * coefficients[2];
    plane->points.push_back(p);
  }
}

/* Upper half, and the left and right lower halves of a pedestrian. */
void compute3ZoneCovarianceMatrix(pcl::PointCloud<pcl::PointXYZI>::Ptr plane, Eigen::Vector4f &mean, float *partial_covariance_2d) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr zone_decomposed[3];
  for(int i = 0; i < 3; i++)
    zone_decomposed[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
  for(size_t i = 0; i < plane->size(); i++) {
    if(plane->points[i].z >= mean(2)) { // upper half
      zone_decomposed[0]->points.push_back(plane->points[i]);
    } else {
      if(plane->points[i].y >= mean(1)) // left lower half
	zone_decomposed[1]->points.push_back(plane->points[i]);
      else // right lower half
	zone_decomposed[2]->points.push_back(plane->points[i]);
    }
  }
  
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  for(int i = 0; i < 3; i++) {
    pcl::compute3DCentroid(*zone_decomposed[i], centroid);
    pcl::computeCovarianceMatrix(*zone_decomposed[i], centroid, covariance);
    partial_covariance_2d[i*3+0] = covariance(0,0);
    partial_covariance_2d[i*3+1] = covariance(0,1);
    partial_covariance_2d[i*3+2] = covariance(1,1);
  }
}

void computeHistogramNormalized(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int horiz_bins, int verti_bins, float *histogram) {
  Eigen::Vector4f min, max, min_box, max_box;
  pcl::getMinMax3D(*pc, min, max);
  float horiz_itv, verti_itv;
  horiz_itv = (max[0]-min[0]>max[1]-min[1]) ? (max[0]-min[0])/horiz_bins : (max[1]-min[1])/horiz_bins;
  verti_itv = (max[2] - min[2])/verti_bins;
  
  for(int i = 0; i < horiz_bins; i++) {
    for(int j = 0; j < verti_bins; j++) {
      if(max[0]-min[0] > max[1]-min[1]) {
	min_box << min[0]+horiz_itv*i, min[1], min[2]+verti_itv*j, 0;
	max_box << min[0]+horiz_itv*(i+1), max[1], min[2]+verti_itv*(j+1), 0;
      } else {
	min_box << min[0], min[1]+horiz_itv*i, min[2]+verti_itv*j, 0;
	max_box << max[0], min[1]+horiz_itv*(i+1), min[2]+verti_itv*(j+1), 0;
      }
      std::vector<int> indices;
      pcl::getPointsInBox(*pc, min_box, max_box, indices);
      histogram[i*verti_bins+j] = (float)indices.size() / (float)pc->size();
    }
  }
}

void computeSlice(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int n, float *slice) {
  Eigen::Vector4f pc_min, pc_max;
  pcl::getMinMax3D(*pc, pc_min, pc_max);
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr blocks[n];
  float itv = (pc_max[2] - pc_min[2]) / n;
  if(itv > 0) {
    for(int i = 0; i < n; i++) {
      blocks[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
    for(unsigned int i = 0, j; i < pc->size(); i++) {
      j = std::min((n-1), (int)((pc->points[i].z - pc_min[2]) / itv));
      blocks[j]->points.push_back(pc->points[i]);
    }
    
    Eigen::Vector4f block_min, block_max;
    for(int i = 0; i < n; i++) {
      if(blocks[i]->size() > 0) {
	// pcl::PCA<pcl::PointXYZI> pca;
	// pcl::PointCloud<pcl::PointXYZI>::Ptr block_projected(new pcl::PointCloud<pcl::PointXYZI>);
	// pca.setInputCloud(blocks[i]);
	// pca.project(*blocks[i], *block_projected);
	pcl::getMinMax3D(*blocks[i], block_min, block_max);
      } else {
	block_min.setZero();
	block_max.setZero();
      }
      slice[i*2] = block_max[0] - block_min[0];
      slice[i*2+1] = block_max[1] - block_min[1];
    }
  } else {
    for(int i = 0; i < 20; i++)
      slice[i] = 0;
  }
}

void computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int bins, float *intensity) {
  float sum = 0, mean = 0, sum_dev = 0;
  float min = FLT_MAX, max = -FLT_MAX;
  for(int i = 0; i < 27; i++)
    intensity[i] = 0;
  
  for(size_t i = 0; i < pc->size(); i++) {
    sum += pc->points[i].intensity;
    min = std::min(min, pc->points[i].intensity);
    max = std::max(max, pc->points[i].intensity);
  }
  mean = sum / pc->size();
  
  for(size_t i = 0; i < pc->size(); i++) {
    sum_dev += (pc->points[i].intensity-mean)*(pc->points[i].intensity-mean);
    int ii = std::min(float(bins-1), std::floor((pc->points[i].intensity-min)/((max-min)/bins)));
    intensity[ii]++;
  }
  intensity[25] = sqrt(sum_dev/pc->size());
  intensity[26] = mean;
}

void Object3dDetector::extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f) {
  Eigen::Vector4f centroid, min, max;
  pcl::compute3DCentroid(*pc, centroid);
  pcl::getMinMax3D(*pc, min, max);
  
  f.centroid = centroid;
  f.min = min;
  f.max = max;
  
  // f1: Number of points included the cluster.
  f.number_points = pc->size();
  // f2: The minimum distance to the cluster.
  f.min_distance = FLT_MAX;
  float d2; //squared Euclidean distance
  for(int i = 0; i < pc->size(); i++) {
    d2 = pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y + pc->points[i].z*pc->points[i].z;
    if(f.min_distance > d2)
      f.min_distance = d2;
  }
  //f.min_distance = sqrt(f.min_distance);
  
  pcl::PCA<pcl::PointXYZI> pca;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_projected(new pcl::PointCloud<pcl::PointXYZI>);
  pca.setInputCloud(pc);
  pca.project(*pc, *pc_projected);
  // f3: 3D covariance matrix of the cluster.
  pcl::computeCovarianceMatrixNormalized(*pc_projected, centroid, f.covariance_3d);
  // f4: The normalized moment of inertia tensor.
  computeMomentOfInertiaTensorNormalized(*pc_projected, f.moment_3d);
  // // Navarro et al. assume that a pedestrian is in an upright position.
  // pcl::PointCloud<pcl::PointXYZI>::Ptr main_plane(new pcl::PointCloud<pcl::PointXYZI>), secondary_plane(new pcl::PointCloud<pcl::PointXYZI>);
  // computeProjectedPlane(pc, pca.getEigenVectors(), 2, centroid, main_plane);
  // computeProjectedPlane(pc, pca.getEigenVectors(), 1, centroid, secondary_plane);
  // // f5: 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
  // compute3ZoneCovarianceMatrix(main_plane, pca.getMean(), f.partial_covariance_2d);
  // // f6 and f7
  // computeHistogramNormalized(main_plane, 7, 14, f.histogram_main_2d);
  // computeHistogramNormalized(secondary_plane, 5, 9, f.histogram_second_2d);
  // f8
  computeSlice(pc, 10, f.slice);
  // f9
  computeIntensity(pc, 25, f.intensity);
}

void Object3dDetector::saveFeature(Feature &f, struct svm_node *x) {
  x[0].index  = 1;  x[0].value  = f.number_points; // libsvm indices start at 1
  x[1].index  = 2;  x[1].value  = f.min_distance;
  x[2].index  = 3;  x[2].value  = f.covariance_3d(0,0);
  x[3].index  = 4;  x[3].value  = f.covariance_3d(0,1);
  x[4].index  = 5;  x[4].value  = f.covariance_3d(0,2);
  x[5].index  = 6;  x[5].value  = f.covariance_3d(1,1);
  x[6].index  = 7;  x[6].value  = f.covariance_3d(1,2);
  x[7].index  = 8;  x[7].value  = f.covariance_3d(2,2);
  x[8].index  = 9;  x[8].value  = f.moment_3d(0,0);
  x[9].index  = 10; x[9].value  = f.moment_3d(0,1);
  x[10].index = 11; x[10].value = f.moment_3d(0,2);
  x[11].index = 12; x[11].value = f.moment_3d(1,1);
  x[12].index = 13; x[12].value = f.moment_3d(1,2);
  x[13].index = 14; x[13].value = f.moment_3d(2,2);
  // for(int i = 0; i < 9; i++) {
  //   x[i+14].index = i+15;
  //   x[i+14].value = f.partial_covariance_2d[i];
  // }
  // for(int i = 0; i < 98; i++) {
  //   x[i+23].index = i+24;
  //   x[i+23].value = f.histogram_main_2d[i];
  // }
  // for(int i = 0; i < 45; i++) {
  //   x[i+121].index = i+122;
  //   x[i+121].value = f.histogram_second_2d[i];
  // }
  for(int i = 0; i < 20; i++) {
    x[i+14].index = i+15;
    x[i+14].value = f.slice[i];
  }
  for(int i = 0; i < 27; i++) {
    x[i+34].index = i+35;
    x[i+34].value = f.intensity[i];
  }
  x[FEATURE_SIZE].index = -1;
  
  // for(int i = 0; i < FEATURE_SIZE; i++) {
  //   std::cerr << x[i].index << ":" << x[i].value << " ";
  //   std::cerr << std::endl;
  // }
}

void Object3dDetector::classify() {
  geometry_msgs::PoseArray pose_array;
  visualization_msgs::MarkerArray marker_array;
  
  for(std::vector<Feature>::iterator it = features_.begin(); it != features_.end(); it++) {
    bool svm_find_human = false;
    
    if(train_round_ > 0) {
      /*** scale data ***/
      saveFeature(*it, svm_node_);
      for(int i = 0; i < FEATURE_SIZE; i++) {
	if(svm_range_[i][0] == svm_range_[i][1]) // skip single-valued attribute
	  continue;
	if(svm_node_[i].value == svm_range_[i][0])
	  svm_node_[i].value = svm_xlower_;
	else if(svm_node_[i].value == svm_range_[i][1])
	  svm_node_[i].value = svm_xupper_;
	else
	  svm_node_[i].value = svm_xlower_ + (svm_xupper_ - svm_xlower_) * (svm_node_[i].value - svm_range_[i][0]) / (svm_range_[i][1] - svm_range_[i][0]);
      }
      
      /*** predict ***/
      if(svm_check_probability_model(svm_model_)) {
	double prob_estimates[svm_model_->nr_class];
	svm_predict_probability(svm_model_, svm_node_, prob_estimates);
	clusters_probability_.push_back(prob_estimates[0]);
	if(prob_estimates[0] > human_probability_)
	  svm_find_human = true;
      } else {
	if(svm_predict(svm_model_, svm_node_) == 1)
	  svm_find_human = true;
      }
    }
    
    /*** cluster pose ***/
    if(train_round_ < max_trains_) {
      geometry_msgs::Pose pose;
      pose.position.x = it->centroid[0];
      pose.position.y = it->centroid[1];
      // using pose.position.z as ID, dirty but the simplest way.
      // @todo message type for online learning: PoseStamped?
      unsigned int i = learnable_clusters_.size()-features_.size()+it-features_.begin();
      pose.position.z = learnable_clusters_[i]->header.seq; // pose ID
      pose.orientation.w = 1;
      pose_array.poses.push_back(pose);
      if(svm_find_human)
	learnable_clusters_[i]->header.frame_id = "human";
      //std::cerr << "pose ID = " << pose.position.z << std::endl;
    } else {
      if(svm_find_human) {
	geometry_msgs::Pose pose;
	pose.position.x = it->centroid[0];
	pose.position.y = it->centroid[1];
	pose.position.z = -1;//it->centroid[2];
	pose.orientation.w = 1;
	pose_array.poses.push_back(pose);
      }
    }
    
    /*** bounding box ***/
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.ns = "object3d";
    marker.id = it-features_.begin();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point p[24];
    p[0].x = it->max[0]; p[0].y = it->max[1]; p[0].z = it->max[2];
    p[1].x = it->min[0]; p[1].y = it->max[1]; p[1].z = it->max[2];
    p[2].x = it->max[0]; p[2].y = it->max[1]; p[2].z = it->max[2];
    p[3].x = it->max[0]; p[3].y = it->min[1]; p[3].z = it->max[2];
    p[4].x = it->max[0]; p[4].y = it->max[1]; p[4].z = it->max[2];
    p[5].x = it->max[0]; p[5].y = it->max[1]; p[5].z = it->min[2];
    p[6].x = it->min[0]; p[6].y = it->min[1]; p[6].z = it->min[2];
    p[7].x = it->max[0]; p[7].y = it->min[1]; p[7].z = it->min[2];
    p[8].x = it->min[0]; p[8].y = it->min[1]; p[8].z = it->min[2];
    p[9].x = it->min[0]; p[9].y = it->max[1]; p[9].z = it->min[2];
    p[10].x = it->min[0]; p[10].y = it->min[1]; p[10].z = it->min[2];
    p[11].x = it->min[0]; p[11].y = it->min[1]; p[11].z = it->max[2];
    p[12].x = it->min[0]; p[12].y = it->max[1]; p[12].z = it->max[2];
    p[13].x = it->min[0]; p[13].y = it->max[1]; p[13].z = it->min[2];
    p[14].x = it->min[0]; p[14].y = it->max[1]; p[14].z = it->max[2];
    p[15].x = it->min[0]; p[15].y = it->min[1]; p[15].z = it->max[2];
    p[16].x = it->max[0]; p[16].y = it->min[1]; p[16].z = it->max[2];
    p[17].x = it->max[0]; p[17].y = it->min[1]; p[17].z = it->min[2];
    p[18].x = it->max[0]; p[18].y = it->min[1]; p[18].z = it->max[2];
    p[19].x = it->min[0]; p[19].y = it->min[1]; p[19].z = it->max[2];
    p[20].x = it->max[0]; p[20].y = it->max[1]; p[20].z = it->min[2];
    p[21].x = it->min[0]; p[21].y = it->max[1]; p[21].z = it->min[2];
    p[22].x = it->max[0]; p[22].y = it->max[1]; p[22].z = it->min[2];
    p[23].x = it->max[0]; p[23].y = it->min[1]; p[23].z = it->min[2];
    for(int i = 0; i < 24; i++)
      marker.points.push_back(p[i]);
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    if(svm_find_human) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 0.5;
      marker.color.b = 1.0;
    }
    marker.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(marker);
  }
  
  /*** publish pose and marker ***/
  if(pose_array.poses.size()) {
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = frame_id_;
    pose_array_pub_.publish(pose_array);
    //std::cerr << "[object3d_detector_ol] published " << pose_array.poses.size() << " poses *** ";
  }
  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }
}

void Object3dDetector::train() {
  if(train_round_ == max_trains_ || (positive_+negative_) < (max_positives_+max_negatives_))
    return;
  
  clock_t t = clock();
  std::cerr << "Training round " << train_round_ << " started" << std::endl;
  
  /*** scale back the previous data ***/
  for(int i = 0; i < (max_positives_+max_negatives_)*train_round_; i++) {
    for(int j = 0; j < FEATURE_SIZE; j++) {
      if(svm_range_[j][0] == svm_range_[j][1]) // skip single-valued attribute
	continue;
      if(svm_problem_.x[i][j].value == svm_xlower_)
	svm_problem_.x[i][j].value = svm_range_[j][0];
      else if(svm_problem_.x[i][j].value == svm_xupper_)
	svm_problem_.x[i][j].value = svm_range_[j][1];
      else
	svm_problem_.x[i][j].value = svm_range_[j][0] + (svm_problem_.x[i][j].value - svm_xlower_) * (svm_range_[j][1] - svm_range_[j][0]) / (svm_xupper_ - svm_xlower_);
    }
  }
  
  /*** save data to file ***/
  // std::ofstream s;
  // s.open("svm_training_data");
  // //s.open(boost::to_string(train_round_).c_str());
  // for(int i = 0; i < svm_problem_.l; i++) {
  //   s << svm_problem_.y[i];
  //   for(int j = 0; j < FEATURE_SIZE; j++)
  //     s << " " << svm_problem_.x[i][j].index << ":" <<  svm_problem_.x[i][j].value;
  //   s << "\n";
  // }
  // s.close();
  
  /*** scale the current data ***/
  for(int i = 0; i < FEATURE_SIZE; i++) {
    svm_range_[i][0] = FLT_MAX; // min range
    svm_range_[i][1] = -FLT_MAX; // max range
  }
  for(int i = 0; i < svm_problem_.l; i++) {
    for(int j = 0; j < FEATURE_SIZE; j++) {
      svm_range_[j][0] = std::min(svm_range_[j][0], (float)svm_problem_.x[i][j].value);
      svm_range_[j][1] = std::max(svm_range_[j][1], (float)svm_problem_.x[i][j].value);
    }
  }
  // for(int i = 0; i < FEATURE_SIZE; i++) // debug print
  //   std::cerr << "svm scale range [attribute " << i << "]: " << svm_range_[i][0] << ", " << svm_range_[i][1] << std::endl;
  for(int i = 0; i < svm_problem_.l; i++) {
    for(int j = 0; j < FEATURE_SIZE; j++) {
      if(svm_range_[j][0] == svm_range_[j][1]) // skip single-valued attribute
	continue;
      if(svm_problem_.x[i][j].value == svm_range_[j][0])
	svm_problem_.x[i][j].value = svm_xlower_;
      else if(svm_problem_.x[i][j].value == svm_range_[j][1])
	svm_problem_.x[i][j].value = svm_xupper_;
      else
	svm_problem_.x[i][j].value = svm_xlower_ + (svm_xupper_ - svm_xlower_) * (svm_problem_.x[i][j].value - svm_range_[j][0]) / (svm_range_[j][1] - svm_range_[j][0]);
      //std::cerr << "training data " << i << " [attribute " << j << "]: " << svm_problem_.x[i][j].value << std::endl;
    }
  }
  
  /*** train ***/
  if(find_the_best_training_parameters_) {
    std::ofstream s;
    s.open("svm_training_data");
    for(int i = 0; i < svm_problem_.l; i++) {
      s << svm_problem_.y[i];
      for(int j = 0; j < FEATURE_SIZE; j++)
	s << " " << svm_problem_.x[i][j].index << ":" <<  svm_problem_.x[i][j].value;
      s << "\n";
    }
    s.close();
    
    std::cerr << "Finding the best training parameters ..." << std::endl;
    if(svm_check_parameter(&svm_problem_, &svm_parameter_) == NULL) {
      char result[100];
      FILE *fp = popen("./grid.py svm_training_data", "r");
      if(fp == NULL) {
	std::cerr << "Can not run cross validation!" << std::endl;
      } else {
	if(fgets(result, 100, fp) != NULL) {
	  char *pch = strtok(result, " ");
	  svm_parameter_.C = atof(pch); pch = strtok(NULL, " ");
	  svm_parameter_.gamma = atof(pch); pch = strtok(NULL, " ");
	  float rate = atof(pch);
	  std::cerr << "Best c=" << svm_parameter_.C << ", g=" << svm_parameter_.gamma << " CV rate=" << rate << std::endl;
	}
      }
      pclose(fp);
    }
  }
  svm_model_ = svm_train(&svm_problem_, &svm_parameter_);
  
  std::cerr << "Training round " << train_round_ << " finished with " << float(clock()-t)/CLOCKS_PER_SEC << " seconds" << std::endl;
  
  /*** reset parameters ***/
  if(train_round_ == 0) {
    max_positives_ = round_positives_;
    max_negatives_ = round_negatives_;
  }
  if(++train_round_ < max_trains_) {
    positive_ = 0;
    negative_ = 0;
  }
  learnable_clusters_.clear();
  clusters_probability_.clear();
  
  /*** debug saving ***/
  // if(svm_save_model("pedestrian.model", svm_model_) == 0)
  //   std::cerr << "A model has been generated here: ~/.ros/pedestrian.model" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "object3d_detector_ol");
  Object3dDetector d;
  /*** single-thread ***/
  ros::spin();
  /*** multi-thread ***/
  //ros::MultiThreadedSpinner spinner(4);
  //spinner.spin();
  return 0;
}
