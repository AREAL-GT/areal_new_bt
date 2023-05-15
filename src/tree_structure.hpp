// tree_structure.hpp
// Author: Ethan Tse
// Date: 03/22/2023
// This is the XML file for the behavior tree.  It needs to be kept in a hpp file instead of an xml file because of the
// way that colcon builds the project.  The xml file is stored as a string in the hpp file and is called in the main function.
static const char* xml_text = R"(
<root BTCPP_format="4" main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
        <Fallback>        
            <Action ID="isWaypointListEmpty" next_waypoint="{waypoint}" finished="{finished}"/>
            <Action ID="FlyToWaypoint" waypoint="{waypoint}"/>
        </Fallback>
    </BehaviorTree>
</root>
 )";


// static const char* xml_text = R"(
//  <root BTCPP_format="4" >
//      <BehaviorTree ID="MainTree">
//         <Sequence name="root_sequence">
//             <Script   code=" location1:='2, 0, -3' " />
//             <Script   code=" location2:='-2, 4, -5' " />
//             <Script   code=" location3:='0, 0, -1' " />
//             <Fly   setpoint="{location1}"/>
//             <Fly   setpoint="{location2}"/>
//             <Fly   setpoint="{location3}"/>
//         </Sequence>
//      </BehaviorTree>
//  </root>
//  )";