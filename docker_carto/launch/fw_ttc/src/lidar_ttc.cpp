#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <string>
#include <sstream>
#include <vector>

#define rad_to_deg(rad) ((rad) * 180.0 / M_PI)

class Freeway_Lidar_Ttc
{
  public:
    Freeway_Lidar_Ttc(ros::NodeHandle *nh)
    {
      if(!n->getParam("scan_topic", scan_topic_)) {
	      ROS_WARN("Could not get value of scan_topic parameter, using default value.");
	      scan_topic_ = "/scan"; // Use a default value if parameter is not set
	      }
      if(!n->getParam("odom_topic", odom_topic_)) {
	      ROS_WARN("Could not get value of odom_topic parameter, using default value.");
	      odom_topic_ = "/odom"; // Use a default value if parameter is not set
	      }
      if(!n->getParam("frequency", frequency_)) {
	      ROS_WARN("Could not get value of frequency parameter, using default value.");
	      frequency_ = 20.0; // Use a default value if parameter is not set
	      }
      if(!n->getParam("range_reversed", range_reversed_)) {
	      ROS_WARN("Could not get value of range_reversed parameter, using default value.");
	      range_reversed_ = true; // Use a default value if parameter is not set
	      }
      if(! n->getParam("signal_distance", signal_distance_)) {
	      ROS_WARN("Could not get value of signal_distance parameter, using default value.");
	      signal_distance_ = 0.4; // Use a default value if parameter is not set
	      }
      if(!n->getParam("signal_distance_teb", signal_distance_teb_)) {
	      ROS_WARN("Could not get value of signal_distance_teb parameter, using default value.");
	      signal_distance_teb_ = 0.45; // Use a default value if parameter is not set
	      }
      if(!n->getParam("signal_release", signal_release_)) {
	      ROS_WARN("Could not get value of signal_release parameter, using default value.");
	      signal_release_ = 0.55; // Use a default value if parameter is not set
	      }
      if(!n->getParam("obstacle_deg", obstacle_deg_)) {
	      ROS_WARN("Could not get value of obstacle_deg parameter, using default value.");
	      obstacle_deg_ = 2; // Use a default value if parameter is not set
	      }
      // if(!n->getParam("angle_min", angle_min_)) {
	    //   ROS_WARN("Could not get value of angle_min parameter, using default value.");
	    //   angle_min_ = -3.14;
	    //   }
      // if(!n->getParam("angle_max", angle_max_)) {
	    //   ROS_WARN("Could not get value of angle_max parameter, using default value.");
	    //   angle_max_ = 3.14;
	    //   }
      if(!n->getParam("ttc_time", ttc_time_)) {
	      ROS_WARN("Could not get value of ttc_time parameter, using default value.");
	      ttc_time_ = 2.0;
	      }  
      if(!n->getParam("ttc_deg", ttc_deg_)) {
	      ROS_WARN("Could not get value of ttc_deg_ parameter, using default value.");
	      ttc_deg_ = 1;
	      }  
      if(!n->getParam("detecting_deg", detecting_deg_)) {
	      ROS_WARN("Could not get value of detecting_deg parameter, using default value.");
	      detecting_deg_ = 120.0;
	      } 
      if(!n->getParam("detecting_deg_ttc", detecting_deg_ttc_)) {
	      ROS_WARN("Could not get value of detecting_deg_ttc parameter, using default value.");
	      detecting_deg_ttc_ = 100.0;
	      }
      n = nh;
      
      input_scan_sub = n->subscribe(scan_topic_, 10, &Freeway_Lidar_Ttc::front_obstacle_update_cb, this);
      input_odom_sub = n->subscribe(odom_topic_, 10, &Freeway_Lidar_Ttc::odom_update_cb, this);
      ttc_pub = n->advertise<std_msgs::Float32>("/freeway/ttc", 10);
      ttc_flag_pub = n->advertise<std_msgs::Bool>("/freeway/ttc_flag", 10);
      obstacle_count_ = 0;
      signal_checker = 0;
      signal2_checker = 0;
      signal_checker_teb = 0;
      signal2_checker_teb = 0;
      isFrontObstacleDetected_data = false;
      isFrontObstacleTebDetected_data = false;
      odom_msg;
    }

    void log_param() {

        ROS_INFO_STREAM("scan_topic: " << scan_topic_);
        ROS_INFO_STREAM("odom_topic: " << odom_topic_);
        ROS_INFO("frequency: %f", frequency_);
        if(range_reversed_) ROS_INFO("range_reversed: true");
	      else ROS_INFO("range_reversed: false");
        ROS_INFO("signal_distance: %f", signal_distance_);
        ROS_INFO("signal_distance_teb: %f", signal_distance_teb_);
        ROS_INFO("signal_release: %f", signal_release_);
        // ROS_INFO("angle_min: %f", angle_min_);
        // ROS_INFO("angle_max: %f", angle_max_);
        ROS_INFO("ttc_time: %f", ttc_time_);
        ROS_INFO("ttc_deg: %d", ttc_deg_);
        ROS_INFO("detecting_deg: %f", detecting_deg_);
        ROS_INFO("obstacle_deg_: %f", obstacle_deg_);
        ROS_INFO("detecting_deg_ttc_: %f", detecting_deg_ttc_);
    }

    bool getIsFrontObstacleDetected() const {
        return isFrontObstacleDetected_data;
    }
    bool getIsFrontObstacleTebDetected() const {
        return isFrontObstacleTebDetected_data;
    }

    float getFrequency() const {
      return frequency_;
    }

  private:
    ros::NodeHandle* n;
    ros::Subscriber input_scan_sub;
    ros::Subscriber input_odom_sub;
    ros::Publisher ttc_pub;
    ros::Publisher ttc_flag_pub;
    std::string scan_topic_;
    std::string odom_topic_;
    nav_msgs::Odometry odom_msg;
    bool range_reversed_;
    bool isFrontObstacleDetected_data;
    bool isFrontObstacleTebDetected_data;
    float obstacle_deg_;
    int obstacle_count_;
    int ttc_deg_;
    double signal_distance_;
    double signal_distance_teb_;
    double signal_release_;
    // double angle_max_;
    // double angle_min_;
    uint32_t signal_checker;
    uint32_t signal2_checker;
    uint32_t signal_checker_teb;
    uint32_t signal2_checker_teb;
    float frequency_;
    float ttc_time_;
    float detecting_deg_;
    float detecting_deg_ttc_;

    void front_obstacle_update_cb(const sensor_msgs::LaserScan::ConstPtr& input_scan) {
      float ran_max = input_scan->range_max;
      double angle_max_ = input_scan->angle_max;
      double angle_min_ = input_scan->angle_min;
	    double detecting_range = rad_to_deg((angle_max_)-(angle_min_));
      float res_per_deg = (int)input_scan->ranges.size()/detecting_range;
      float las_mid_ran = res_per_deg * (detecting_range/2.0);
      int detecting_deg_ran_d2 = (int)(floor((detecting_deg_*res_per_deg)/2.0));
      int detecting_deg_ran = (int)(floor(detecting_deg_*res_per_deg));
      //float deg_15 = floor(res_per_deg * (float)15.0);
      std::vector<float> ran_arr;
      ran_arr.reserve((int)(floor(detecting_deg_*res_per_deg))+10);
      //float* ran_arr = new float[(int)(floor(detecting_deg_*res_per_deg))]();
      obstacle_count_ = (int)(floor(obstacle_deg_*res_per_deg));
      nav_msgs::Odometry odom_now;
      odom_now = odom_msg;
      signal_checker = 0;
      signal_checker_teb = 0;
      signal2_checker = 0;
      signal2_checker_teb = 0;
      int count_ignore = 0;

      if(range_reversed_) {
        if (!input_scan->ranges.empty()) {

            for (unsigned int i = 0; i < detecting_deg_ran_d2; i++) {
                if(input_scan->ranges[i] < signal_distance_ && input_scan->ranges[i] > 0.0) {
                  signal_checker++;
                  }
    	          if(input_scan->ranges[i] < signal_distance_teb_ && input_scan->ranges[i] > 0.0) {
                  signal_checker_teb++;
                  }

                if(input_scan->ranges[i] == 0.0 || input_scan->ranges[i] > ran_max ||std::isnan(input_scan->ranges[i]) || std::isinf(input_scan->ranges[i])) count_ignore++;
                ran_arr.push_back(input_scan->ranges[i]);
                //ran_arr[i]=input_scan->ranges[i];
              }

            for (unsigned int i = input_scan->ranges.size()-1 - detecting_deg_ran_d2; i < input_scan->ranges.size()-1; i++) {
                if(input_scan->ranges[i] < signal_distance_ && input_scan->ranges[i] > 0.0 ) {
                  signal_checker++;
                }
    	          if(input_scan->ranges[i] < signal_distance_teb_ && input_scan->ranges[i] > 0.0) {
    	            signal_checker_teb++;
    	          }
                if (signal_checker >= obstacle_count_) {
                  signal_checker = obstacle_count_;
                  //ROS_INFO("Emergency_Safety_LiDAR Detection!!!!!!!!!!!!!!!!!!!!");
                  }
    	          if (signal_checker_teb >= obstacle_count_) {
    	            signal_checker_teb = obstacle_count_;
    	            }

                if(input_scan->ranges[i] == 0.0 || input_scan->ranges[i] > ran_max ||std::isnan(input_scan->ranges[i]) || std::isinf(input_scan->ranges[i])) count_ignore++;
                ran_arr.push_back(input_scan->ranges[i]);
                //ran_arr[(i+((int)(floor((detecting_deg_*res_per_deg)/2.0))))-(input_scan->ranges.size()-1 - (int)(floor((detecting_deg_*res_per_deg)/2.0)))]=input_scan->ranges[i];
              }

              for (unsigned int i = 0; i < detecting_deg_ran-1; i++) {
                if (ran_arr[i] >= signal_release_ && ran_arr[i] <= ran_max) {
                  signal2_checker++;
    	            signal2_checker_teb++;
                  }
                }
                if (signal2_checker >= detecting_deg_ran-2-count_ignore) {
                  signal2_checker = 0;
                  signal_checker = 0;
    	            signal2_checker_teb = 0;
    	            signal_checker_teb = 0;
                }
                else if (signal2_checker < detecting_deg_ran-2-count_ignore) { signal2_checker = 0; signal2_checker_teb = 0; }

                calc_ttc(ran_arr, res_per_deg, ran_max, odom_now);

                isFrontObstacleDetected_data = isFrontObstacleDetected(signal_checker, obstacle_count_);
                isFrontObstacleTebDetected_data = isFrontObstacleTebDetected(signal_checker_teb, obstacle_count_);

                //delete [] ran_arr;
               }
          }
      
      else if(!range_reversed_) {
           if (!input_scan->ranges.empty()) {
          
               for (unsigned int i = int(floor(las_mid_ran)) - detecting_deg_ran_d2; i < int(floor(las_mid_ran)) + detecting_deg_ran_d2; i++) {
                if(input_scan->ranges[i] < signal_distance_ && input_scan->ranges[i] > 0.0) {
                  signal_checker++;
                  }
                if(input_scan->ranges[i] < signal_distance_teb_ && input_scan->ranges[i] > 0.0) {
                  signal_checker_teb++;
                  }

                if (signal_checker >= obstacle_count_) {
                   signal_checker = obstacle_count_;
                  }

                if (signal_checker_teb >= obstacle_count_) {
                   signal_checker_teb = obstacle_count_;
                  }

                if(input_scan->ranges[i] == 0.0 || input_scan->ranges[i] > ran_max ||std::isnan(input_scan->ranges[i]) || std::isinf(input_scan->ranges[i])) count_ignore++;
                ran_arr.push_back(input_scan->ranges[i]);
                //ran_arr[i-(int(floor(las_mid_ran))-(int)(floor((detecting_deg_*res_per_deg)/2.0)))]=input_scan->ranges[i];
                //if(ran_arr[i] == 0.0) count_ignore++;
	              //if(ran_arr[i-(int(floor(las_mid_ran))-(int)(floor((detecting_deg_*res_per_deg)/2.0)))] == 0.0) count_ignore++;
	                //ROS_INFO("ran_arr %d", i-(int(floor(las_mid_ran))-(int)(floor((detecting_deg_*res_per_deg)/2.0))));
	                //ROS_INFO(" %f", ran_arr[i-(int(floor(las_mid_ran))-(int)(floor((detecting_deg_*res_per_deg)/2.0)))]);
               } 
	             //ROS_INFO("count_ignore : %d", count_ignore);
               for (unsigned int i = 0; i < detecting_deg_ran-1; i++) {
                if (ran_arr[i] >= signal_release_ && ran_arr[i] <= ran_max) {
                  signal2_checker++;
                  signal2_checker_teb++;
                }
               }

               if (signal2_checker >= detecting_deg_ran-2-count_ignore) {
                signal2_checker = 0;
                signal_checker = 0;
   	            signal2_checker_teb = 0;
   	            signal_checker_teb = 0;
               }

               else if (signal2_checker < detecting_deg_ran-2-count_ignore) { signal2_checker = 0; signal2_checker_teb = 0; }

               calc_ttc(ran_arr, res_per_deg, ran_max, odom_now);

               isFrontObstacleDetected_data = isFrontObstacleDetected(signal_checker, obstacle_count_);
               isFrontObstacleTebDetected_data = isFrontObstacleTebDetected(signal_checker_teb, obstacle_count_);

               //delete [] ran_arr;
              }
           }
       }

      void calc_ttc(std::vector<float>& ran_arr_, float res_per_deg, float ran_max_, nav_msgs::Odometry odom) {
        std_msgs::Float32 ttc;
        std_msgs::Bool ttc_flag;
        uint32_t ttc_count = 0;
        float min_ttc = std::numeric_limits<float>::max();
        int _ttc_deg_ = int(floor(res_per_deg * this->ttc_deg_));
        float _detecting_deg_ttc_ = int(floor((res_per_deg * detecting_deg_ttc_)/2.0));

        if (odom.twist.twist.linear.x < 0.0) {
          odom.twist.twist.linear.x = 0.0;
        }

        for(int i = floor((ran_arr_.size()/2.0)-_detecting_deg_ttc_); i < floor((ran_arr_.size()/2.0)+_detecting_deg_ttc_); i++) {
          if (!std::isnan(ran_arr_[i]) && !std::isinf(ran_arr_[i]) && !(ran_arr_[i] > ran_max_) && !(ran_arr_[i] == 0.0)) {
            if (odom.twist.twist.linear.x != 0.0) {
              float time_to_collision = (ran_arr_[i] / odom.twist.twist.linear.x);
              if(time_to_collision < min_ttc) min_ttc = time_to_collision; // update only when smaller
              if(time_to_collision < ttc_time_) ttc_count++;
            }
          }
        }

        ttc.data = min_ttc;
        // Only publish a time to collision message if the minimum value seen is less than 1.0
        if (ttc_count >= _ttc_deg_) ttc_flag.data = true;
        else ttc_flag.data = false;

        ttc_flag_pub.publish(ttc_flag);
        ttc_pub.publish(ttc);
      }

      bool isFrontObstacleDetected(uint32_t _signal_checker, int _obstacle_count_)
      {
        return (_signal_checker >= _obstacle_count_-1);
      }

      bool isFrontObstacleTebDetected(uint32_t _signal_checker_teb, int _obstacle_count_)
      {
        return (_signal_checker_teb >= _obstacle_count_-1);
      }

      void odom_update_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        this->odom_msg = *msg;
      }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "freeway_lidar_ttc_node");
  ros::NodeHandle n;
  Freeway_Lidar_Ttc FLT = Freeway_Lidar_Ttc(&n);  
  ros::Publisher front_obstacle_pub = n.advertise<std_msgs::Bool>("/freeway/front_obstacle", 10);
  ros::Publisher front_obstacle_teb_pub = n.advertise<std_msgs::Bool>("/freeway/front_obstacle_teb", 10);
  FLT.log_param();
  float frequency = FLT.getFrequency();

  ros::Rate loop_rate(frequency);

  while (ros::ok())
  {
    std_msgs::Bool front_obstacle_detected;
    std_msgs::Bool front_obstacle_teb_detected;

    front_obstacle_detected.data = FLT.getIsFrontObstacleDetected();
    front_obstacle_teb_detected.data = FLT.getIsFrontObstacleTebDetected();

    front_obstacle_pub.publish(front_obstacle_detected);
    front_obstacle_teb_pub.publish(front_obstacle_teb_detected);    

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
