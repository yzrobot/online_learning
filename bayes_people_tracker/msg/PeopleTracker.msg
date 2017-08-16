std_msgs/Header header
string[] uuids             # Unique uuid5 (NAMESPACE_DNS) person id as string. Id is based on system time on start-up and tracker id. Array index matches ids array index
geometry_msgs/Pose[] poses # The real world poses of the detected people in the given target frame. Default: /map. Array index matches ids/uuids array index
float64[] distances        # The distances of the detected persons to the robot (polar coordinates). Array index matches ids array index.
float64[] angles           # Angles of the detected persons to the coordinate frames z axis (polar coordinates). Array index matches ids array index.
float64 min_distance       # The minimal distance in the distances array.
float64 min_distance_angle # The angle according to the minimal distance.
