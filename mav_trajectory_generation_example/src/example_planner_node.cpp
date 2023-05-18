/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include <mav_trajectory_generation_example/example_planner_node.h>

ExamplePlannerNode::ExamplePlannerNode()
{
  // Subscribe to "waypoint" topic
  waypoint_sub_ = nh_.subscribe("waypoint", 1, &ExamplePlannerNode::waypointCallback, this);

  // Subscribe to "odom" topic
  odom_sub_ = nh_.subscribe("odom", 1, &ExamplePlannerNode::odomCallback, this);

  // Publish to "trajectory" topic
  trajectory_pub_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 1);
}

void ExamplePlannerNode::waypointCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  // Callback function for "waypoint" topic
  // define set point
  wp_position << msg->vector.x, msg->vector.y, msg->vector.z;
  wp_velocity << 0.0, 0.0, 0.0;
  planner.planTrajectory(wp_position, wp_velocity, &trajectory);
  planner.publishTrajectory(trajectory);
}

void ExamplePlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Callback function for "odom" topic
  // This callback function is empty.
  // You can add your own logic here if needed.
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ExamplePlannerNode example_planner_node;

  // ExamplePlanner planner;
  // ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  // ros::Duration(5.0).sleep();
  // ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // // define set point
  // Eigen::Vector3d position, velocity;
  // position << 0.0, 1.0, 2.0;
  // velocity << 0.0, 0.0, 0.0;

  // // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  // ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  // std::cin.get();
  // for (int i = 0; i < 10; i++) {
  //   ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  // }

  // mav_trajectory_generation::Trajectory trajectory;
  // planner.planTrajectory(position, velocity, &trajectory);
  // planner.publishTrajectory(trajectory);
  // ROS_WARN_STREAM("DONE. GOODBYE.");

  ros::Rate rate(100); // Set the loop rate to 50Hz

  while (ros::ok())
  {
    ros::spinOnce();

    // Your main logic here

    rate.sleep();
  }

  return 0;
}