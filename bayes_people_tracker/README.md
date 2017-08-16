## People Tracker
This package uses the bayes_tracking library developed by Nicola Bellotto (University of Lincoln), please cite with: [10.5281/zenodo.15825](https://zenodo.org/record/15825) and [1]

The people_tracker uses a single config file to add an arbitrary amount of detectors. The file `config/detectors.yaml` contains the necessary information for the upper_body_detector and the ROS leg_detector (see `to_pose_array` in detector_msg_to_pose_array/README.md):

```
bayes_people_tracker:
    filter_type: "UKF"                                         # The Kalman filter type: EKF = Extended Kalman Filter, UKF = Uncented Kalman Filter
    cv_noise_params:                                           # The noise for the constant velocity prediction model
        x: 1.4
        y: 1.4
    detectors:                                                 # Add detectors under this namespace
        upper_body_detector:                                   # Name of detector (used internally to identify them). Has to be unique.
            topic: "/upper_body_detector/bounding_box_centres" # The topic on which the geometry_msgs/PoseArray is published
            cartesian_noise_params:                            # The noise for the cartesian observation model
                x: 0.5
                y: 0.5
            matching_algorithm: "NNJPDA"                       # The algorthim to match different detections. NN = Nearest Neighbour, NNJPDA = NN Joint Probability Data Association
        leg_detector:                                          # Name of detector (used internally to identify them). Has to be unique.
            topic: "/to_pose_array/leg_detector"               # The topic on which the geometry_msgs/PoseArray is published
            cartesian_noise_params:                            # The noise for the cartesian observation model
                x: 0.2
                y: 0.2
            matching_algorithm: "NNJPDA"                       # The algorthim to match different detections. NN = Nearest Neighbour, NNJPDA = NN Joint Probability Data Association
```

New detectors are added under the parameter namespace `bayes_people_tracker/detectors`. Let's have a look at the upper body detector as an example:

### Tracker Parameters

The tracker offers two configuration parameters:
* `filter_type`: This specefies which variant of the Kalman filter to use. Currently, it implements an Extended and an Unscented Kalman filter which can be chosen via `EKF` and `UKF`, respectively.
* `cv_noise_params`: parameter is used for the constant velocity prediction model.
 * specifies the standard deviation of the x and y velocity.

### Detector Parameters

* For every detector you have to create a new namespace where the name is used as an internal identifier for this detector. Therefore it has to be unique. In this case it is `upper_body_detector`
* The `topic` parameter specifies the topic under which the detections are published. The type has to be `geometry_msgs/PoseArray`. See `to_pose_array` in detector_msg_to_pose_array/README.md if your detector does not publish a PoseArray.
* The `cartesian_noise_params` parameter is used for the Cartesian observation model.
 * specifies the standard deviation of x and y.
* `matching_algorithm` specifies the algorithm used to match detections from different sensors/detectors. Currently there are two different algorithms which are based on the Mahalanobis distance of the detections (default being NNJPDA if parameter is misspelled):
 * NN: Nearest Neighbour
 * NNJPDA: Nearest Neighbour Joint Probability Data Association

All of these are just normal ROS parameters and can be either specified by the parameter server or using the yaml file in the provided launch file.

### Message Type:

The tracker publishes the following:
* `pose`: A `geometry_msgs/PoseStamped` for the clostes person.
* `pose_array`: A `geometry_msgs/PoseArray` for all detections.
* `people`: A `people_msgs/People` for all detections. Can be used for layerd costmaps.
* `marker_array`: A `visualization_msgs/MarkerArray` showing little stick figures for every detection. Figures are orient according to the direction of velocity.
* `trajectory`: A `geometry_msgs/PoseArray` for human trajectory.
* `positions`: A `bayes_people_tracker/PeopleTracker` message. See below. 

```
std_msgs/Header header
string[] uuids             # Unique uuid5 (NAMESPACE_DNS) person id as string. Id is based on system time on start-up and tracker id. Array index matches ids array index
geometry_msgs/Pose[] poses # The real world poses of the detected people in the given target frame. Default: /map. Array index matches ids/uuids array index
float64[] distances        # The distances of the detected persons to the robot (polar coordinates). Array index matches ids array index.
float64[] angles           # Angles of the detected persons to the coordinate frames z axis (polar coordinates). Array index matches ids array index.
float64 min_distance       # The minimal distance in the distances array.
float64 min_distance_angle # The angle according to the minimal distance.
```

The poses will be published in a given `target_frame` (see below) but the distances and angles will always be relative to the robot in the `/base_link` tf frame.

### Running
Parameters:

* `target_frame`: _Default: /base_link_:the tf frame in which the tracked poses will be published. 
* `position`: _Default: /people_tracker/positions_: The topic under which the results are published as bayes_people_tracker/PeopleTracker`
* `pose`: _Default: /people_tracker/pose_: The topic under which the closest detected person is published as a geometry_msgs/PoseStamped`
* `pose_array`: _Default: /people_tracker/pose_array_: The topic under which the detections are published as a geometry_msgs/PoseArray`
* `poeple`: _Default: /people_tracker/people_: The topic under which the results are published as people_msgs/People`
* `marker`: _Default /people_tracker/marker_array_: A visualisation marker array.

You can run the node with:

```
roslaunch bayes_people_tracker people_tracker.launch
```

This is the recommended way of launching it since this will also read the config file and set the right parameters for the detectors.

[1] N. Bellotto and H. Hu, “Computationally efficient solutions for tracking people with a mobile robot: an experimental evaluation of bayesian filters,” Autonomous Robots, vol. 28, no. 4, pp. 425–438, 2010.
