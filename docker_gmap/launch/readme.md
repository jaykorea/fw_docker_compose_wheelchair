Option 1.

(1) launch the file "launch_d455_ekf_pub.launch"
  - It launches realsense d455 camera with gyro, aceel enable
  - It launches imu madgwik filtere to filter imu data from /d400/imu topic
  - It launches robot_localization pkg publishing odometry/filtered topic combined with /odom_md /imu/data topic with publish_tf option enabled

(2) launch the file named "angular_bounds_filter.launch"
  - Node filteres /scan_yd topic data to get specific angular data

(3) launch the file "fw_mapping.launch" with arg rviz:=true option to use visualization pkg
  - It launches "rtabmap.launch" to load rtabmap & rgbd_odometry node with localization arg option set false
  * See arg & params to optimize rtabmap ros node

(4) launch the file "fw_navigation.launch" with arg rviz:=true option to use visualization pkg
  - It launches "rtabmap.launch" to load rtabmap & rgbd_odometry node with localization arg option set true
  * See arg & params to optimize rtabmap ros node

Option 2.


(1) launch the file "launch_d455_ekf_nopub.launch"
  - It launches realsense d455 camera with gyro, aceel enable
  - It launches imu madgwik filtere to filter imu data from /d400/imu topic
  - It launches robot_localization pkg publishing odometry/filtered topic combined with /odom_md /imu/data topic with publish_tf option disabled

(2) launch the file named "angular_bounds_filter.launch"
  - Node filteres /scan_yd topic data to get specific angular data

(3) launch the file "fw_mapping_vio.launch" with arg rviz:=true option to use visualization pkg
  - It launches "rtabmap.launch" to load rtabmap & rgbd_odometry node with localization arg option set false
  * See arg & params to optimize rtabmap ros node

(4) launch the file "fw_navigation_vio.launch" with arg rviz:=true option to use visualization pkg
  - It launches "rtabmap.launch" to load rtabmap & rgbd_odometry node with localization arg option set true
  * See arg & params to optimize rtabmap ros node

