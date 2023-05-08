#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "freeway_msgs/DistanceTimeCalculator.h"
#include <boost/bind.hpp>
#include <string>
#include <sstream>

#define rad_to_deg(rad) ((rad) * 180.0 / M_PI)

class Freeway_Safety
{
  public:
    Freeway_Safety(ros::NodeHandle *nh)
    {
      if(!n->getParam("kp", kp_)) {
	      ROS_WARN("Could not get value of kp parameter, using default value.");
	      kp_ = 10.0; // Use a default value if parameter is not set
	      }
      if(!n->getParam("ki", ki_)) {
	      ROS_WARN("Could not get value of ki parameter, using default value.");
	      ki_ = 5.0; // Use a default value if parameter is not set
	      }
      if(!n->getParam("kd", kd_)) {
	      ROS_WARN("Could not get value of kd parameter, using default value.");
	      kd_ = 1.0; // Use a default value if parameter is not set
	      }
      if(!n->getParam("ttc_desired", ttc_desired_)) {
	      ROS_WARN("Could not get value of ttc_desired parameter, using default value.");
	      ttc_desired_ = 5.0; // Use a default value if parameter is not set
	      }
      if(!n->getParam("ttc_limit", ttc_limit_)) {
	      ROS_WARN("Could not get value of ttc_limit parameter, using default value.");
	      ttc_limit_ = 5.0; // Use a default value if parameter is not set
	      }
      if(!n->getParam("frequency", frequency_)) {
	      ROS_WARN("Could not get value of frequency parameter, using default value.");
	      frequency_ = 5.0; // Use a default value if parameter is not set
	      }
      if(!n->getParam("odom_topic", odom_topic_)) {
	      ROS_WARN("Could not get value of odom_topic parameter, using default value.");
	      odom_topic_ = "/odom"; // Use a default value if parameter is not set
	      }

      n = nh;

      ttc_sub = n->subscribe("/freeway/ttc", 10, &Freeway_Safety::ttc_update_cb, this);
      ttc_flag_sub = n->subscribe("/freeway/ttc_flag", 10, &Freeway_Safety::ttc_flag_update_cb, this);
      front_obstacle_sub = n->subscribe("/freeway/front_obstacle", 10, &Freeway_Safety::front_obstacle_update_cb, this);
      dtc_sub = n->subscribe("/freeway/distancetimecalculator", 10, &Freeway_Safety::dtc_update_cb, this);
      odom_sub = n->subscribe(odom_topic_, 10, &Freeway_Safety::odom_update_cb, this);

      cmd_e_vel_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel/emer", 10);
      resume_pub = n->advertise<std_msgs::Empty>("/freeway/resume", 10);
      move_base_cancel_pub = n->advertise<actionlib_msgs::GoalID>("move_base_flex/move_base/cancel", 10);
      brake_stop_pub = n->advertise<std_msgs::Empty>("/freeway/initsetbrakestop", 1);
      clearcostmap_pub = n->serviceClient<std_srvs::Empty>("move_base_flex/clear_costmaps");

      odom_msg;
      timer;

      ttc_current_ = 10.0;
      desired_vel_ = 0.0;
      error = 0.0;
      integral = 0.0;
      derivative = 0.0;
      prev_error = 0.0;
      dt_ = 1.0/frequency_;
  
      ttc_time = 10.0;
      ttc_flag = false;
      ttc_activate_ = false;
      front_obstacle_detected = false;

      e_stop_flag = true;
    }

    void log_param() {

        ROS_INFO_STREAM("odom_topic: " << odom_topic_);
        ROS_INFO("frequency: %f", frequency_);

        ROS_INFO("kp: %f", kp_);
        ROS_INFO("ki: %f", ki_);
        ROS_INFO("kd: %f", kd_);
        ROS_INFO("ttc_desired: %f", ttc_desired_);
    }

    float getFrequency() const {
      return frequency_;
    }

    float getTtcTime() const {
        return ttc_time;
    }

    void resume_tim_cb(const ros::TimerEvent&, ros::Publisher& pub) {
        std_msgs::Empty resume_msg;
        pub.publish(resume_msg);
    }

    nav_msgs::Odometry getOdomData() const {
        return odom_msg;
    }

    void slow_down(float ttc_time, nav_msgs::Odometry odom) {

      // Calculate the error between the desired ttc and the current ttc
      ttc_current_ = ttc_time;
      error = ttc_desired_ - ttc_current_;  
      // Calculate the integral and derivative terms
      integral += error * dt_;
      derivative = (error - prev_error) / dt_;  
      // Calculate the output of the PID controller
      float output = kp_ * error + ki_ * integral + kd_ * derivative;  
      // Calculate the desired velocity based on the output
      desired_vel_ = std::max(0.0f, static_cast<float>(odom.twist.twist.linear.x - output));
    
      // Apply the desired velocity as a linear acceleration to the robot
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = desired_vel_;
      if (odom.twist.twist.linear.x != 0) {
        cmd_e_vel_pub.publish(cmd_vel);
      }
      // Update the previous error for the next iteration
      prev_error = error;
    }
    
    void brake_robot(nav_msgs::Odometry odom) {
      actionlib_msgs::GoalID empty_goal;
      std_msgs::Empty brake_msg;
      geometry_msgs::Twist zero_velocity;
      zero_velocity = geometry_msgs::Twist();
      move_base_cancel_pub.publish(empty_goal);
      while (odom.twist.twist.linear.x > 0.0) {
        brake_stop_pub.publish(brake_msg);
        //cmd_e_vel_pub.publish(zero_velocity);
      }
      e_stop_flag = true;
    }
    
    void stop_logic(float ttc_time, nav_msgs::Odometry odom) {
      if (ttc_activate_) {
        // Slow down the robot to increase TTC to at least 2 seconds
        slow_down(ttc_time, odom);
        if ( e_stop_flag == false && front_obstacle_detected == true) {
        // Stop the robot if a front obstacle is detected
        brake_robot(odom);
        } 
        else if (e_stop_flag == true && front_obstacle_detected == false) {
          // Resume normal operation if no front obstacle is detected
          std_srvs::Empty srv;
          clearcostmap_pub.call(srv);
          timer = n->createTimer(ros::Duration(2.7), boost::bind(&Freeway_Safety::resume_tim_cb, this, _1, resume_pub), true);
          e_stop_flag = false;
        }
      }
      if(odom.twist.twist.linear.x <= 0.0) ttc_activate_ = false;
    }

  private:
    ros::NodeHandle* n;
    ros::Subscriber ttc_sub;
    ros::Subscriber ttc_flag_sub;
    ros::Subscriber front_obstacle_sub;
    ros::Subscriber dtc_sub;
    ros::Subscriber odom_sub;

    ros::Publisher cmd_e_vel_pub;
    ros::Publisher resume_pub;
    ros::Publisher move_base_cancel_pub;
    ros::Publisher brake_stop_pub;
    ros::ServiceClient clearcostmap_pub;

    ros::Timer timer;

    std::string odom_topic_;
    nav_msgs::Odometry odom_msg;
    freeway_msgs::DistanceTimeCalculator dtc;

    float kp_;
    float ki_;
    float kd_;
    float ttc_desired_;
    float ttc_limit_;
    float ttc_current_;
    float desired_vel_;
    float error;
    float integral;
    float derivative;
    float prev_error;
    float dt_;

    float ttc_time;
    float ttc_flag;
    bool front_obstacle_detected;
    bool e_stop_flag;
    bool ttc_activate_;

    float frequency_;

    void ttc_update_cb(const std_msgs::Float32::ConstPtr& msg) {
        ttc_time = msg->data;
        if (ttc_time <= ttc_limit_) ttc_activate_ = true;
    }
    void ttc_flag_update_cb(const std_msgs::Bool::ConstPtr& msg) {
        ttc_flag = msg->data;
    }
    void front_obstacle_update_cb(const std_msgs::Bool::ConstPtr& msg) {
        front_obstacle_detected = msg->data;
    }
    void dtc_update_cb(const freeway_msgs::DistanceTimeCalculator::ConstPtr& msg) {
        dtc = *msg;
    }
    void odom_update_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_msg = *msg;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "freeway_lidar_ttc_node");
  ros::NodeHandle n;
  Freeway_Safety FS = Freeway_Safety(&n);  
  FS.log_param();
  float frequency = FS.getFrequency();

  ros::Rate loop_rate(frequency);

  while (ros::ok())
  {
    // Get the TTC time and odometry data
    float ttc_time = FS.getTtcTime();
    nav_msgs::Odometry odom = FS.getOdomData();

    // Execute the stop logic
    FS.stop_logic(ttc_time, odom);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
