// main.cpp
// Author: Ethan Tse
// Date: 01/23/2023
//
// This is the main file for the behavior tree which registers the action leaf node and creates the tree from the xml file.

#include "change_pos_client.hpp"
#include "tree_structure.hpp"
#include "leafs.hpp"

// Main loop to make file executable, should break out into main.cpp and .hpp
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::list<std::array<float,3>> waypoint_list;
    //add 4 waypoints
    waypoint_list.push_back({1,2,-3});
    waypoint_list.push_back({-4,5,-6});
    waypoint_list.push_back({-3,-1,-2});
    waypoint_list.push_back({5,5,-9});

    BehaviorTreeFactory factory;
    factory.registerNodeType<FlyToWaypointLeaf>("FlyToWaypoint");
    factory.registerNodeType<isWaypointListEmpty>("isWaypointListEmpty", &waypoint_list);
    auto tree = factory.createTreeFromText(xml_text);
    
    do{
        tree.tickWhileRunning();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));        
    }while(tree.rootBlackboard()->get<bool>("finished") == false);

    rclcpp::shutdown();

    return 0;


}