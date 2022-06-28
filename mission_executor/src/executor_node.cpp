#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "mavros_msgs/srv/command_tol.hpp"
#include "drone_msgs/msg/mission.hpp"
#include "drone_msgs/msg/task.hpp"
#include "drone_msgs/action/execute_mission.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "mission_executor/visibility_control.h"

namespace mission_executor {
  class ExecutorNode : public rclcpp::Node {
    public:
      using ExecuteMission = drone_msgs::action::ExecuteMission;
      using GoalHandleExecuteMission = rclcpp_action::ServerGoalHandle<ExecuteMission>;

      MISSION_EXECUTOR_PUBLIC 
      explicit ExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("executor_node", options) {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<ExecuteMission>(
          this,
          "execute_mission",
          std::bind(&ExecutorNode::handle_goal, this, _1, _2),
          std::bind(&ExecutorNode::handle_cancel, this, _1),
          std::bind(&ExecutorNode::handle_accepted, this, _1));
      }

    private:
      rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;
      rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_landing_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

      rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, 
        std::shared_ptr<const ExecuteMission::Goal> goal) {
          RCLCPP_INFO(this->get_logger(), "Received execute mission request of length %d", goal->mission.tasks.size());
          (void)uuid;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }

      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteMission> goal_handle
      ) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      }

      void handle_accepted(const std::shared_ptr<GoalHandleExecuteMission> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ExecutorNode::execute, this, _1), goal_handle}.detach();
      }

      void execute(const std::shared_ptr<GoalHandleExecuteMission> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing mission");

        const auto goal = goal_handle->get_goal();
        const auto &mission = goal->mission;
        const auto &start_index = goal->start_index;

        auto feedback = std::make_shared<ExecuteMission::Feedback>();        
        auto result = std::make_shared<ExecuteMission::Result>();

        size_t i = start_index;
        size_t task_length = mission.tasks.size();
        int error_count = 0;

        auto current_task = mission.tasks[0];
        ExecuteMission::Result current_task_result;

        while (i < task_length && error_count < 3) {
          // handle cancellation
          if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Execution cancelled");
            result->result = ExecuteMission::Result::EXECUTION_CANCELLED;
            result->error_string = std::string("execution cancelled");
            goal_handle->canceled(result);
            return;
          }

          current_task = mission.tasks[i];

          feedback->current_task = current_task;
          feedback->current_task_index = i;
          feedback->current_task_result = ExecuteMission::Result::EXECUTION_STARTING;

          // publish pre-execution feedback
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Pre-execution feedback published");

          // execute task
          try
          {
            current_task_result = execute_task(current_task);
          }
          catch(const std::exception& e)
          {
            RCLCPP_ERROR(this->get_logger(), e.what());
            current_task_result = ExecuteMission::Result();
            current_task_result.result = ExecuteMission::Result::EXECUTION_FAILED;
            current_task_result.error_string = std::string(e.what());
          }
          
          feedback->current_task = current_task;
          feedback->current_task_index = i;
          feedback->current_task_result = current_task_result.result;
          feedback->current_task_error_string = current_task_result.error_string;

          // publish post-execution feedback
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Post-execution feedback published");

          if (current_task_result.result < 0) {
            ++error_count;
          } else {
            ++i;
            error_count = 0;
          }
        }

        if (rclcpp::ok()) {
          if (error_count >= 3) {
            result->result = current_task_result.result;
            result->error_string = current_task_result.error_string;
          } else {
            result->result = ExecuteMission::Result::EXECUTION_SUCCESS;
          }

          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Execution finished");
        }
      }

      ExecuteMission::Result execute_task(const drone_msgs::msg::Task& task) {
        ExecuteMission::Result execution_result;

        switch (task.type) {
          case drone_msgs::msg::Task::TAKEOFF:
            execution_result = takeoff(task.integer_args, task.float_args, task.string_args);
            break;

          case drone_msgs::msg::Task::LAND:
            break;

          case drone_msgs::msg::Task::WAYPOINT:
            break;

          case drone_msgs::msg::Task::RTH:
            break;
          
          case drone_msgs::msg::Task::SCAN_BARCODE:
            break;

          case drone_msgs::msg::Task::HOVER:
            break;

          case drone_msgs::msg::Task::DOCK:
            break;
          
          default:
            execution_result = ExecuteMission::Result();
            execution_result.result = ExecuteMission::Result::ARGUMENT_ERROR;
            execution_result.error_string = "unknown task type " + task.type;
            break;
        }

        return execution_result;
      }

      ExecuteMission::Result takeoff(const std::vector<int64_t> integer_args, std::vector<double> float_args, std::vector<std::string> string_args) {
        using namespace std::chrono_literals;

        ExecuteMission::Result execution_result = ExecuteMission::Result();

        auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        takeoff_request->min_pitch = 0.0f;
        takeoff_request->yaw = 0.0f;
        takeoff_request->latitude = 0.0f;
        takeoff_request->longitude = 0.0f;
        takeoff_request->altitude = float_args[0];

        auto takeoff_result = this->takeoff_landing_client_->async_send_request(takeoff_request);
        takeoff_result.wait_for(5s);

        if (takeoff_result.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Takeoff success");
          execution_result.result = ExecuteMission::Result::EXECUTION_SUCCESS;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Takeoff failed");
          execution_result.result = ExecuteMission::Result::EXECUTION_FAILED;
          execution_result.error_string = std::string("");
        }

        return execution_result;
      }
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(mission_executor::ExecutorNode)