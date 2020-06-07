#ifndef GBPLANNER_TRAJOPT_H
#define GBPLANNER_TRAJOPT_H

#include <iostream>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class GbplannerTrajOpt {
 public:
  GbplannerTrajOpt(const ros::NodeHandle& nh);

  std::vector<geometry_msgs::Pose> smoothPath(
      const std::vector<geometry_msgs::Pose>& line_path);

  bool optimizeTrajectory(const std::vector<geometry_msgs::Pose>& path,
                          mav_trajectory_generation::Trajectory* trajectory);

  void publishTrajectory(
      const mav_trajectory_generation::Trajectory& trajectory);

  bool interpolatePath(const std::vector<Eigen::Vector3d>& path,
                       double discrete_length,
                       std::vector<Eigen::Vector3d>& path_intp);

  virtual bool isPathInCollision(const Eigen::Vector3d& start,
                                 const Eigen::Vector3d& end) const = 0;
  virtual bool isCollision(const Eigen::Vector3d& p) const = 0;

 private:
  ros::NodeHandle nh_;
  ros::Publisher traj_opt_pub_;
  ros::Publisher pub_trajectory_;
  std::string world_frame_id_;
};

#endif  // GBPLANNER_TRAJOPT_H
