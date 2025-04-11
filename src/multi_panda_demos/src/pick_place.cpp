#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <thread>
#include <chrono>
#include <vector>
#include <stdexcept>

using namespace std::chrono_literals;

class PickPlaceController
{
public:
  PickPlaceController(const rclcpp::Node::SharedPtr &node)
    : node_(node),
      move_group_(node, "dual_panda_arm"),
      planning_scene_interface_()
  {
    move_group_.setMaxVelocityScalingFactor(0.25);
    move_group_.startStateMonitor();
  }

  void execute()
  {
    spawnObjects();  // 👈 Spawn SDF-based objects

    waitForValidState();

    geometry_msgs::msg::PoseStamped pick_pose;
    pick_pose.header.frame_id = "world";
    pick_pose.pose.position.x = 0.4;
    pick_pose.pose.position.y = 0.2;
    pick_pose.pose.position.z = 0.4;
    pick_pose.pose.orientation.w = 1.0;

    move_group_.setPoseTarget(pick_pose, "left_panda_link8");
    planAndExecute("Pick pose");

    std::this_thread::sleep_for(1s);
    RCLCPP_INFO(node_->get_logger(), "Simulated pick action");

    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "world";
    place_pose.pose.position.x = 0.6;
    place_pose.pose.position.y = -0.2;
    place_pose.pose.position.z = 0.4;
    place_pose.pose.orientation.w = 1.0;

    move_group_.setPoseTarget(place_pose, "left_panda_link8");
    planAndExecute("Place pose");

    std::this_thread::sleep_for(1s);
    RCLCPP_INFO(node_->get_logger(), "Simulated place action");
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  void waitForValidState()
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for valid initial robot state...");
    auto current_state = move_group_.getCurrentState(10.0);
    if (!current_state)
    {
      throw std::runtime_error("Robot did not provide valid state within timeout");
    }
  }

  void planAndExecute(const std::string &description)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "%s: Planning successful, executing...", description.c_str());
      auto result = move_group_.execute(plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        throw std::runtime_error(description + " execution failed.");
      }
      RCLCPP_INFO(node_->get_logger(), "%s: Execution complete", description.c_str());
    }
    else
    {
      throw std::runtime_error(description + " planning failed.");
    }
  }

  void spawnObjects()
  {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // Vertical Stick 1
    moveit_msgs::msg::CollisionObject stick1;
    stick1.header.frame_id = "world";
    stick1.id = "vertical_stick_1";

    shape_msgs::msg::SolidPrimitive stick_shape;
    stick_shape.type = stick_shape.BOX;
    stick_shape.dimensions = {0.02, 0.02, 0.2};

    geometry_msgs::msg::Pose stick1_pose;
    stick1_pose.position.x = 0.35;
    stick1_pose.position.y = -0.25;
    stick1_pose.position.z = 0.1;
    stick1_pose.orientation.w = 1.0;

    stick1.primitives.push_back(stick_shape);
    stick1.primitive_poses.push_back(stick1_pose);
    stick1.operation = stick1.ADD;
    collision_objects.push_back(stick1);

    // Vertical Stick 2
    moveit_msgs::msg::CollisionObject stick2 = stick1;
    stick2.id = "vertical_stick_2";
    stick2.primitive_poses[0].position.y = 0.25;
    collision_objects.push_back(stick2);

    planning_scene_interface_.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(node_->get_logger(), "Spawned SDF-based collision objects.");
  }
};

int main(int argc, char **argv)
{
  try
  {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options = rclcpp::NodeOptions().allow_undeclared_parameters(true);
    auto node = std::make_shared<rclcpp::Node>("pick_place_node", options);

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    while (!parameters_client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for parameter service.");
        return 1;
      }
      RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameter service...");
    }

    auto parameter_list = parameters_client->list_parameters({"robot_description_kinematics"}, 10);
    auto parameters = parameters_client->get_parameters(parameter_list.names);
    node->set_parameters(parameters);

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    std::thread executor_thread([&executor]() { executor->spin(); });

    PickPlaceController controller(node);
    controller.execute();

    executor->cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  catch (...)
  {
    std::cerr << "Unknown fatal error occurred." << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
