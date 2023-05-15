// leafs.hpp
// Author: Ethan Tse
// Date: 03/22/2023
//
// This holds the leaf definitions for the behavior tree

#include "behaviortree_cpp/bt_factory.h"
using namespace BT;
#define MY_STRING @MY_STRING@
#include <iostream>
#include <list>

class FlyToWaypointLeaf : public BT::SyncActionNode
{
public:
  FlyToWaypointLeaf(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config)
  { }

  static PortsList providedPorts()
  {
    return { InputPort<std::array<float, 3>>("waypoint") };
  }
    
  NodeStatus tick() override
  {
    std::cout << "Fly to waypoint" << std::endl;
    
    auto bt_action_client = 
        std::make_shared<areal_new_bt::ChangePosClient>();

    auto res = getInput<std::array<float, 3>>("waypoint");
    if(!res)
    {
        throw RuntimeError("missing required input [waypoint]: ", res.error() );
    }

    std::array<float, 3> waypoint = res.value();

    std::cout <<"waypoint: (" << waypoint[0] << ", " << waypoint[1] << ", " << waypoint[2] << ")" << std::endl;

    bt_action_client->goto_setpoint(waypoint);

    while(!bt_action_client->is_goal_done())
    {
        
        rclcpp::spin_some(bt_action_client);
        
    }
   
    return NodeStatus::SUCCESS;
  }


};


// SyncActionNode (synchronous action) with an input port.
class isWaypointListEmpty : public SyncActionNode
{
public:
  // If your Node has ports, you must use this constructor signature 
  isWaypointListEmpty(const std::string& name, const NodeConfig& config, std::list<std::array<float,3>> *waypoint_list)
    : SyncActionNode(name, config), waypoint_list(waypoint_list)
  { }

  // It is mandatory to define this STATIC method.
  static PortsList providedPorts()
  {
    // This action has a single input port called "message"
    return { OutputPort<std::array<float, 3>>("next_waypoint"), OutputPort<bool>("finished")};
  }

  // Override the virtual function tick()
  NodeStatus tick() override
  {
    //check if waypoint_list is empty
    int queue_size = waypoint_list->size();
    std::cout << "Queue at: " << queue_size<< std::endl;
    //if not empty, get the next waypoint and set it as output
    if (queue_size > 0){
        std::cout << "Queue is not empty" << std::endl;
        setOutput("finished", false);
        std::array<float,3> next_waypoint = waypoint_list->front();
        waypoint_list->pop_front();
        setOutput("next_waypoint", next_waypoint);
        return NodeStatus::FAILURE;
    }
    std::cout << "Queue is empty" << std::endl;
    setOutput("finished", true);

    return NodeStatus::SUCCESS;
  }

  private:
    std::list<std::array<float,3>> *waypoint_list; // shared information
};


// Converts a string to float vector.
namespace BT
{
    template <> inline std::array<float, 3> convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ',');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            std::array<float, 3> output;
            output[0]     = convertFromString<float>(parts[0]);
            output[1]     = convertFromString<float>(parts[1]);
            output[2]     = convertFromString<float>(parts[2]);
            return output;
        }
    }
} // end namespace BT