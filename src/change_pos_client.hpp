// change_pos_client.cpp
// Date: 12/15/2022
// Author: Ethan Tse
//
// A ROS2 Node that calls the px4_pos_set_move action server to fly to a setpoint
// Adapted from bt_autoland_client.cpp written by Adam Garlow
//



// Client includes
#include <functional>
#include <future>

#include "areal_landing_uav_interfaces/action/px4_pos_set_move.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

namespace areal_new_bt
{

class ChangePosClient : public rclcpp::Node
{
public:

    // Set up aliases to simplify notation for action interfaces
    using PX4PosSetMove = areal_landing_uav_interfaces::action::PX4PosSetMove;
    using GoalHandlePX4PosSetMove = rclcpp_action::ClientGoalHandle<PX4PosSetMove>;

    // constructor
    explicit ChangePosClient(const rclcpp::NodeOptions &node_options = 
        rclcpp::NodeOptions()) : Node("behavior_tree_sequencer", node_options), 
        goal_done_(false)
    {

        // Create client for autoland action
        this->client_ptr_ = rclcpp_action::create_client<PX4PosSetMove>(this, 
            "px4_pos_set_move");

 
    }

    // Callback method for timer to send BT autoland goal message
    void send_goal()
    {
        using namespace std::placeholders; // Placeholders for callback methods

        this->timer_->cancel(); // Cancel timer so only called once

        this->goal_done_ = false; // Set goal to not done

        // Check to see if action client initialized
        if (!this->client_ptr_) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
            this->goal_done_ = true; // Set true to end action
            return;
        }

        // Cancel the client request if action server does not respond
        if (!this->client_ptr_->wait_for_action_server(
            std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "PX4PosSetMove action server not "
                "avaliable after waiting");
            this->goal_done_ = true; // Set true to end action
            return;
        }

        // Set up autoland goal message
        auto goal_msg = PX4PosSetMove::Goal();
        //  
        // std::array<float, 3> my_setpoint = {5.0f, 0.0f, -5.0f};
        goal_msg.position_set = desired_setpoint;
        goal_msg.tolerance = 0.1f; //taken from bt_autoland.hpp

        RCLCPP_INFO(this->get_logger(), "Sending autoland goal");

        // Set up goal send options
        auto send_goal_options = 
            rclcpp_action::Client<PX4PosSetMove>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&ChangePosClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&ChangePosClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&ChangePosClient::result_callback, this, _1);
        
        // Send autoland message to begin bt activation
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, 
            send_goal_options);

    } // send_goal

    // Public check to see if goal is done
    bool is_goal_done() const
    {
        return this->goal_done_;
    }

    void goto_setpoint(std::array<float, 3> setpoint)
    {
        desired_setpoint = setpoint;
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(0),
                        std::bind(&ChangePosClient::send_goal, this));
        // A timer is necessary to get the action client to work. 
        // Without it, the action client will not send the goal message when spin_some is called 
    }

private:

    // Declare timer and autoland action client
    rclcpp_action::Client<PX4PosSetMove>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool goal_done_; // Goal done boolean
    std::array<float, 3> desired_setpoint; // Desired setpoint

    // Define goal response callback method (accept or reject goal)
    void goal_response_callback(
        std::shared_future<GoalHandlePX4PosSetMove::SharedPtr> future)
    {

        auto goal_handle = future.get(); 

        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Move goal rejected by "
                "PX4PosSetMove server");
        }else
        {
            RCLCPP_INFO(this->get_logger(), "Move goal accepted by "
                "PX4PosSetMove server");
        }

    } // goal_response_callback

    // Define feedback callback method (feedback from action)
    void feedback_callback(GoalHandlePX4PosSetMove::SharedPtr, 
        const std::shared_ptr<const PX4PosSetMove::Feedback> feedback)
    {

        RCLCPP_INFO(this->get_logger(), "action feedback "
            "placeholder");

    } // feedback_callback

    // Define result callback method (process result of bt action)
    void result_callback(const GoalHandlePX4PosSetMove::WrappedResult &result)
    {

        this->goal_done_ = true; // Set to true as action is over

        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:

                RCLCPP_INFO(this->get_logger(), "Action success");
                break;

            case rclcpp_action::ResultCode::ABORTED:

                RCLCPP_ERROR(this->get_logger(), "Action goal "
                    "aborted");
                return;

            case rclcpp_action::ResultCode::CANCELED:

                RCLCPP_ERROR(this->get_logger(), "Action goal "
                    "cancled");
                return;

            default:

                RCLCPP_ERROR(this->get_logger(), "Action unknown "
                    "result code");
                return;

        }

        // Print some result info if desired
        RCLCPP_INFO(this->get_logger(), "Successfully flew to: " + 
        std::to_string(desired_setpoint[0]) + ", " + std::to_string(desired_setpoint[1]) + ", " + std::to_string(desired_setpoint[2]));

    } // result_callback

}; // class ChangePosClient

} // namespace areal_landing_bt_autoland



