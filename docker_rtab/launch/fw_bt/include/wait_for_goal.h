#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <movebase_client.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>

//static geometry_msgs::PoseStamped received_goal;
bool goal_trigger;

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    printf("Received goal callback pose x: %f, y: %f\n", msg->pose.position.x, msg->pose.position.y);
    goal_trigger = true;
    
}

class WaitForGoal : public BT::SyncActionNode
{
    public:
        WaitForGoal(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            sub_ = node_.subscribe("move_base_simple/goal", 10, goal_cb);
        }

        ~WaitForGoal() = default;

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<Pose2D>("goal_out") };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            bool expect_trigger = true;

                    //printf("Checking interrupt....\n");
                    // if (!getInput<std::string>("goal_trigger", expect_trigger)) {
                    //     throw BT::RuntimeError("missing required input [expect_trigger]");
                    // }
            
                geometry_msgs::PoseStampedConstPtr target_pose_once =
                ros::topic::waitForMessage<geometry_msgs::PoseStamped>("move_base_simple/goal");
                if (!target_pose_once)
                {
                  return BT::NodeStatus::FAILURE;
                }
    
                ROS_INFO("newTargetPoseReceived: new goal received!");
    
                geometry_msgs::PoseStamped target_pose = *target_pose_once;
                Pose2D goal_in;
                goal_in.x = target_pose.pose.position.x;
                goal_in.y = target_pose.pose.position.y;
                goal_in.quaternion_z = target_pose.pose.orientation.z;
                goal_in.quaternion_w = target_pose.pose.orientation.w;

                BT::TreeNode::setOutput("goal_out", goal_in);

                ros::spinOnce();

                // if (goal_trigger == expect_trigger) {
                //     goal_trigger = false;
                //     return BT::NodeStatus::FAILURE;
                // }
                // else if (goal_trigger != expect_trigger) {
                //     goal_trigger = false;
                //     BT::TreeNode::setOutput("goal_out", goal_in);
                //     //return BT::NodeStatus::SUCCESS;
                //     //throw BT::RuntimeError("missing required input [goal_in]");

                // }
                //else return BT::NodeStatus::FAILURE;
            return BT::NodeStatus::SUCCESS;
        }

    private:
        ros::NodeHandle node_;
        ros::Subscriber sub_;
};
