#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_NODE_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_trajectory_generation_example/example_planner.h>


class ExamplePlannerNode {
  public:
    ExamplePlannerNode();

  private:
    ros::NodeHandle nh_;
    ExamplePlanner planner;

    ros::Subscriber waypoint_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher trajectory_pub_;

    void waypointCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    mav_trajectory_generation::Trajectory trajectory;
    Eigen::Vector3d wp_position, wp_velocity;
};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_NODE_H
