#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <gbplanner_trajopt/gbplanner_trajopt.h>

GbplannerTrajOpt::GbplannerTrajOpt(const ros::NodeHandle& nh):
  nh_(nh),
  world_frame_id_("world") {
  traj_opt_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "opt_trajectory_markers", 0);
  pub_trajectory_ =
      nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
}

std::vector<geometry_msgs::Pose> GbplannerTrajOpt::smoothPath(
    const std::vector<geometry_msgs::Pose>& line_path) {
  // Prune
  std::vector<Eigen::Vector3d> line_path_vec, line_path_vec_edited;
  for(const auto &p:line_path) {
    Eigen::Vector3d path_point(p.position.x, p.position.y, p.position.z);
    line_path_vec.push_back(path_point);
  }
  line_path_vec_edited.push_back(line_path_vec[0]);
  for(int i=1;i<line_path_vec.size();i++) {
    bool v_status = isPathInCollision(line_path_vec_edited[line_path_vec_edited.size()-1], line_path_vec[i]);
    if(!v_status)
      continue;
    else
      line_path_vec_edited.push_back(line_path_vec[i-1]);
  }
  line_path_vec_edited.push_back(line_path_vec.back());
  // Interpolate
  std::vector<Eigen::Vector3d> line_path_vec_intp;
  const double kIntpLen = 1.0; //m
  interpolatePath(line_path_vec_edited, kIntpLen, line_path_vec_intp);
  // Prune very short segments due to interpolation.
  std::vector<Eigen::Vector3d> line_path_vec_intp_cleaned;
  const double kSegmentLenMin = 0.1;
  line_path_vec_intp_cleaned.push_back(line_path_vec_intp[0]);
  Eigen::Vector3d prev_vec;
  prev_vec = line_path_vec_intp[0];
  for (int i = 1; i < line_path_vec_intp.size(); ++i) {
    Eigen::Vector3d vec = line_path_vec_intp[i] - prev_vec;
    double segment_len = vec.norm();
    if (std::abs(segment_len) > kSegmentLenMin) {
      line_path_vec_intp_cleaned.push_back(line_path_vec_intp[i]);
      prev_vec = line_path_vec_intp[i];
    }
  }

  // Convert to pose
  // std::vector<geometry_msgs::Pose> line_path_new;
  // for (int i = 0; i < line_path_vec_intp_cleaned.size(); ++i) {
  //   geometry_msgs::Pose pose;
  //   pose.position.x = line_path_vec_intp_cleaned[i].x();
  //   pose.position.y = line_path_vec_intp_cleaned[i].y();
  //   pose.position.z = line_path_vec_intp_cleaned[i].z();
  //   pose.orientation.x = 0;
  //   pose.orientation.y = 0;
  //   pose.orientation.z = 0;
  //   pose.orientation.w = 1;
  //   line_path_new.push_back(pose);
  // }

  std::vector<geometry_msgs::Pose> line_path_new;
  for (int i = 0; i < line_path_vec_edited.size(); ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = line_path_vec_edited[i].x();
    pose.position.y = line_path_vec_edited[i].y();
    pose.position.z = line_path_vec_edited[i].z();
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    line_path_new.push_back(pose);
  }

  // std::vector<geometry_msgs::Pose> line_path_new = line_path;

  ROS_WARN("Global path before pruning [%d], after pruning [%d], after interpolating [%d]",
              line_path.size(), line_path_vec_edited.size(), line_path_vec_intp_cleaned.size());

  // Optimize loop
  bool stop_opt = false;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory traj;
  mav_msgs::EigenTrajectoryPoint::Vector traj_points;
  trajectory_msgs::MultiDOFJointTrajectory msg_pub;
  double dt = 0.2;

  int n_max = 5;
  while ((--n_max > 0) && (!stop_opt)) {
    // Optimize
    if (optimizeTrajectory(line_path_new, &trajectory)) {
      // Collision check
      // Sample and convert
      mav_planning_msgs::PolynomialTrajectory4D msg;
      mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
      bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(msg, &traj);
      // double dt = global_planner_->map_manager_->getVoxelStatus();
      std::vector<double> segment_times = trajectory.getSegmentTimes();
      bool path_free = true;

      double time_from_start = 0;
      const double kEpsilon = 0.001;
      int bad_segment = 0;
      for (int i = 0; i < segment_times.size(); ++i) {
        sampleTrajectoryInRange(traj, time_from_start, time_from_start + segment_times[i] - kEpsilon, dt, &traj_points);
        time_from_start += segment_times[i];
        // mav_trajectory_generation::sampleWholeTrajectory(traj, dt, &traj_points);
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(traj_points, &msg_pub);

        auto p1 = msg_pub.points[0].transforms[0];
        Eigen::Vector3d p_start(p1.translation.x, p1.translation.y, p1.translation.z);
        for (int i = 1; i < msg_pub.points.size(); ++i) {
          auto p2 = msg_pub.points[i].transforms[0];
          Eigen::Vector3d p_end(p2.translation.x, p2.translation.y, p2.translation.z);
          double d_diff = (p_start - p_end).norm();
          if (d_diff < 0.1)
            continue;
          if (isCollision(p_end)) {
            path_free = false;
            break;
          }
          p_start = p_end;
        }
        if (!path_free) {
          bad_segment = i;
          break;
        }
      }

      if (path_free) {
        // Good to go.
        ROS_WARN("Collision free -->  execute this path.");
        stop_opt = true;
      } else {
        //
        ROS_WARN("Collision detected --> continue to refine.");
        // add a middle point
        geometry_msgs::Pose pose;
        pose.position.x = 0.5*(line_path_new[bad_segment].position.x + line_path_new[bad_segment+1].position.x);
        pose.position.y = 0.5*(line_path_new[bad_segment].position.y + line_path_new[bad_segment+1].position.y);
        pose.position.z = 0.5*(line_path_new[bad_segment].position.z + line_path_new[bad_segment+1].position.z);
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        line_path_new.insert(line_path_new.begin() + bad_segment + 1, pose);
      }
      publishTrajectory(trajectory);
    }
  }

  if (stop_opt) {
    mav_trajectory_generation::sampleWholeTrajectory(traj, dt, &traj_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(traj_points, &msg_pub);
    std::vector<geometry_msgs::Pose> res;
    for (int i = 0; (i+1) < msg_pub.points.size(); ++i) {
      auto p1 = msg_pub.points[i].transforms[0];
      geometry_msgs::Pose pose;
      pose.position.x = p1.translation.x;
      pose.position.y = p1.translation.y;
      pose.position.z = p1.translation.z;
      pose.orientation.x = p1.rotation.x;
      pose.orientation.y = p1.rotation.y;
      pose.orientation.z = p1.rotation.z;
      pose.orientation.w = p1.rotation.w;
      res.push_back(pose);
    }
    return res;
  } else {
    return line_path_new; //line_path;
  }
}

bool GbplannerTrajOpt::optimizeTrajectory(
    const std::vector<geometry_msgs::Pose>& path,
    mav_trajectory_generation::Trajectory* trajectory) {
  clock_t time_start = clock();
  if (path.size() < 2) return false; // at least start + end point
  ROS_WARN("Optimizing a path with %d segments.", (int)path.size());
  const int dimension = 3;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  if (path.size() == 2) {
    start.makeStartOrEnd(Eigen::Vector3d(path[0].position.x, path[0].position.y, path[0].position.z), derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 0);
  } else {
    start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                          Eigen::Vector3d(path[0].position.x, path[0].position.y, path[0].position.z));
  }
  vertices.push_back(start);

  // intermidiate points
  for (int i = 1; i < path.size()-1; ++i) {
    mav_trajectory_generation::Vertex middle(dimension);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                        Eigen::Vector3d(path[i].position.x, path[i].position.y, path[i].position.z));
    vertices.push_back(middle);
  }

  /******* Configure end point *******/
  if (path.size() == 2) {
    end.makeStartOrEnd(Eigen::Vector3d(path.back().position.x, path.back().position.y, path.back().position.z),
                     derivative_to_optimize);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 0);
  } else {
    end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                       Eigen::Vector3d(path.back().position.x, path.back().position.y, path.back().position.z));
  }
  vertices.push_back(end);

  // estimate initial segment times
  std::vector<double> segment_times;
  const double max_a_ = 1.0;
  const double max_v_= 1.0;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  nlopt::result opt_status = (nlopt::result)opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  ROS_INFO("Opt time [%s]: %f (sec)",
           (opt_status != nlopt::FAILURE) ? "SUCCESS" : "FAILED",
           ((double)(clock() - time_start)) / CLOCKS_PER_SEC);
  if (opt_status != nlopt::FAILURE) return true;
  return false;
}

void GbplannerTrajOpt::publishTrajectory(
    const mav_trajectory_generation::Trajectory& trajectory) {
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               world_frame_id_,
                                               &markers);
  traj_opt_pub_.publish(markers);
}

bool GbplannerTrajOpt::interpolatePath(
    const std::vector<Eigen::Vector3d>& path, double discrete_length,
    std::vector<Eigen::Vector3d>& path_intp) {
  path_intp.clear();
  if (discrete_length <= 0) return false;

  int path_size = path.size();
  if (path_size == 0) {
    return false;
  } else if (path_size == 1) {
    path_intp.push_back(path[0]);
    return true;
  }

  for (int i = 0; i < (path_size - 1); ++i) {
    // Interpolate along the segment.
    Eigen::Vector3d vec = path[i + 1] - path[i];
    double segment_len = vec.norm();
    if (std::abs(segment_len) < 0.01) {
      // Duplicated nodes. Add one only.
      path_intp.push_back(path[i]);
    } else {
      int n = (int)(segment_len / discrete_length);
      Eigen::Vector3d uvec = vec / segment_len * discrete_length;
      for (int j = 0; j <= n; ++j) {
        path_intp.push_back(path[i] + j * uvec);
      }
    }
  }
  return true;
}