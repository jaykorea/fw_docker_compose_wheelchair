#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

// static std::string ok_msg;
bool cancel_trigger = false;

void cancel_cb(const std_msgs::Empty::ConstPtr& msg)
{
    printf("Received freeway/bt/cancel msg");
    cancel_trigger = true;
}

class WaitForOk : public BT::SyncActionNode
{
    public:
        WaitForOk(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            sub_ = node_.subscribe("freeway/bt/cancel", 10, cancel_cb);
        }

        ~WaitForOk() = default;

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("ok_check") };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            std::string expect_ok;

            ros::spinOnce();

            //printf("Checking interrupt....\n");
            if (!getInput<std::string>("ok_check", expect_ok)) {
                throw BT::RuntimeError("missing required input [expect_ok]");
            }
            
            std_msgs::StringConstPtr ok_once =
            ros::topic::waitForMessage<std_msgs::String>("freeway/ok", ros::Duration(1.0));

            if (ok_once) {
                std_msgs::String ok_msg = *ok_once;
                if (ok_msg.data == expect_ok)
                    return BT::NodeStatus::SUCCESS;
                else
                    return BT::NodeStatus::FAILURE;
            }
            else if (cancel_trigger) {
                cancel_trigger = false;
                return BT::NodeStatus::FAILURE;
            }//ok_msg = "";
        }

    private:
        ros::NodeHandle node_;
        ros::Subscriber sub_;
};
