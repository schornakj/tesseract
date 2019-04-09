/**
 * @file trajopt_array_planner.h
 * @brief Tesseract ROS TrajOpt Freespace planner
 *
 * @author Matthew Powelson
 * @date February 27, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_PLANNING_TRAJOPT_SPLINE_PLANNER_H
#define TESSERACT_PLANNING_TRAJOPT_SPLINE_PLANNER_H

#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_planning/basic_planner.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <tesseract_planning/waypoint_definitions.h>

namespace tesseract
{
namespace tesseract_planning
{

/**
 * @brief Config to setup array planner. The input is a vector of waypoints
 *
 * While there are many parameters that can be set, the majority of them have defaults that will be suitable for many
 * problems. These are always required: kin_, env_, end_effector
 */
struct TrajOptSplinePlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajOptSplinePlannerConfig() {}
  virtual ~TrajOptSplinePlannerConfig() {}
  /** @brief Vector of target waypoints given as constraints to the optimization */
  std::vector<WaypointPtr> target_waypoints_;

  /** @brief Selects the type of initialization used for raster path. If GIVEN_TRAJ, then the seed_trajectory_ must be
   * set */
  trajopt::InitInfo::Type init_type_ = trajopt::InitInfo::STATIONARY;
  /** @brief The trajectory used as the optimization seed when init_type_ is set to GIVEN_TRAJ */
  trajopt::TrajArray seed_trajectory_;

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check_ = true;
  /** @brief If true, use continuous collision checking */
  bool collision_continuous_ = true;
  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin_ = 0.025;
  /** @brief If true, a joint velocity cost with a target of 0 will be applied for all timesteps Default: true*/
  bool smooth_velocities_ = true;
  /** @brief If true, a joint acceleration cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_accelerations_ = true;
  /** @brief If true, a joint jerk cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_jerks_ = true;
  /** @brief If true, add a callback to plot each iteration */
  bool plot_callback_ = false;

  /** @brief Basic Kinematics object. ***REQUIRED*** */
  tesseract::BasicKinConstPtr kin_;
  /** @brief Basic Environment object. ***REQUIRED*** */
  tesseract::BasicEnvConstPtr env_;
  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator_;
  /** @brief This is the tip link in the kinematics object used for the cartesian positions ***REQUIRED*** */
  std::string link_;
  /** @brief A transform applied to link_ used for the cartesian position */
  Eigen::Isometry3d tcp_ = Eigen::Isometry3d::Identity();

  Eigen::Vector3d pos_coeffs_ = Eigen::Vector3d(1, 1, 1);

  Eigen::Vector3d rot_coeffs_ = Eigen::Vector3d(1, 1, 1);

  /** @brief Optimization parameters to be used (Optional) */
  sco::BasicTrustRegionSQPParameters params_;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  std::vector<sco::Optimizer::Callback> callbacks_;
};

/**
 * @brief This planner is intended to provide an easy to use interface to TrajOpt for array planning. It takes an array
 * of target waypoints and automates the creation and solve of the TrajOpt Problem
 */
class TrajOptSplinePlanner : public BasicPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajOptSplinePlanner()
  {
    name_ = "TRAJOPT_SPLINE";

    // Error Status Codes
    status_code_map_[-1] = "Invalid config data format";
    status_code_map_[-2] = "Failed to parse config data";
    status_code_map_[-3] = "";

    // Converge Status Codes
  }

  // TODO: Not sure what to do here, but this is defined in the base as virtual
  bool solve(PlannerResponse& response) override { return false; };
  /**
   * @brief Sets up the TrajOpt problem then solves using TrajOptPlanner
   *
   *
   * Note: This does not use the request information because everything is provided by config parameter
   *
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param config Configures the freespace planner
   * @return true if optimization complete
   */
  bool solve(PlannerResponse& response, const TrajOptSplinePlannerConfig& config);

  bool terminate() override;

  void clear() override;
};
}  // namespace tesseract_planning
}
#endif// namespace tesseract#endif  // TESSERACT_PLANNING_TRAJOPT_SPLINE_PLANNER_H
