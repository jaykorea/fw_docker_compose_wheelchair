#include "ros/package.h"
#include "movebase_client.h"
#include "chk_low_battery.h"
#include "always_running.h"
#include "interrupt_event.h"
#include "send_cmd_vel.h"
#include "wait_for_goal.h"
#include "wait_for_ok.h"
#ifdef SUPPORT_OPENVINO
  #include "openvino_event.h"
#endif
#define ZMQ_FOUND true
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

using namespace BT;

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_bt");

  ros::NodeHandle nh("~");
  std::string xml_filename;
  std::string root_path = ros::package::getPath("fw_bt");
  std:: string xml_path = root_path.append("/cfg/bt_navigation.xml");
  nh.param<std::string>("file", xml_filename, xml_path);
  ROS_INFO("Loading XML : %s", xml_filename.c_str());

  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;
  factory.registerNodeType<WaitForGoal>("WaitForGoal");
  factory.registerNodeType<WaitForOk>("WaitForOk");
  factory.registerNodeType<MoveBase>("MoveBase");
  // factory.registerNodeType<SendCommandVel>("SendCommandVel");
  //factory.registerSimpleCondition("CheckBattery", CheckBattery, {BT::InputPort<int>("wait_tick")});
  //factory.registerNodeType<AlwaysRunning>("AlwaysRunning");
  
  // factory.registerSimpleAction("SaySomething2", SaySomethingSimple, 
  //                                say_something_ports );
#ifdef SUPPORT_OPENVINO
  factory.registerNodeType<OpenVINOEvent>("OpenVINOEvent");
#endif

  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

#ifdef ZMQ_FOUND
    // This logger publish status changes using ZeroMQ. Used by Groot
    BT::PublisherZMQ publisher_zmq(tree);
#endif
  // Create a logger
  StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {
    status = tree.rootNode()->executeTick();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
