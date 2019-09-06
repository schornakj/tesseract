/**
 * @file ompl_freespace_planner.h
 * @brief Tesseract OMPL freespace planner.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{
class OMPLFreespacePlannerStatusCategory;

template <typename PlannerSettingsType>
struct OMPLFreespacePlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OMPLFreespacePlannerConfig() {}
  /** @brief Determines the constraint placed at the start of the trajectory */
  Waypoint::Ptr start_waypoint;
  /** @brief Determines the constraint placed at the end of the trajectory */
  Waypoint::Ptr end_waypoint;

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check = true;
  /** @brief If true, use continuous collision checking */
  bool collision_continuous = true;
  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin = 0.025;
  /** @brief Max planning time allowed */
  double planning_time = 5.0;
  /** @brief The max number of solutions. If max solutions are hit it will exit even if other threads are running. */
  int max_solutions = 10;
  /** @brief The number of threads to use */
  int num_threads = 1;
  /** @brief Simplify trajectory */
  bool simplify = true;
  /** @brief This scales the variables search space. Must be same size as number of joints.
   *         If empty it defaults to all ones */
  Eigen::VectorXd weights;

  /** @brief Set the resolution at which state validity needs to be
   * verified in order for a motion between two states to be considered valid. */
  double longest_valid_segment_fraction = 0.01;  // 1%

  /** @brief Planner settings */
  PlannerSettingsType settings;

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract;

  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator;

  /** @brief The ompl state validity checker. If nullptr it uses OMPLFreespacePlanner::isStateValid. */
  ompl::base::StateValidityCheckerFn svc;

  /** @brief The ompl motion validator. If nullptr and continuous collision checking enabled it used
   * ContinuousMotionValidator */
  ompl::base::MotionValidatorPtr mv;
};

/**
 * @brief This planner is intended to provide an easy to use interface to OMPL for freespace planning. It is made to
 * take a start and end point and automate the generation of the OMPL problem.
 */
template <typename PlannerType, typename PlannerSettingsType>
class OMPLFreespacePlanner : public MotionPlanner
{
public:
  /** @brief Construct a planner */
  OMPLFreespacePlanner(std::string name = "OMPL_FREESPACE");

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(const OMPLFreespacePlannerConfig<PlannerSettingsType>& config);

  /**
   * @brief Sets up the OMPL problem then solves. It is intended to simplify setting up
   * and solving freespace motion problems.
   *
   * This planner (and the associated config passed to the setConfiguration) does not expose all of the available
   * configuration data in OMPL. This is done to simplify the interface. However, many problems may require more
   * specific setups. In that case, the source code for this planner may be used as an example.
   *
   * Note: This does not use the request information because everything is provided by config parameter
   *
   * @param response The results of OMPL.
   * @return true if valid solution was found
   */
  tesseract_common::StatusCode solve(PlannerResponse& response) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

private:
  ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space) const;

protected:
  /** @brief The ompl planner planner */
  std::shared_ptr<OMPLFreespacePlannerConfig<PlannerSettingsType>> config_;

  /** @brief The planners status codes */
  std::shared_ptr<const OMPLFreespacePlannerStatusCategory> status_category_;

  /** @brief The ompl planner motion validator */
  ompl::base::MotionValidatorPtr motion_validator_;

  /** @brief The ompl planner simple setup */
  ompl::geometric::SimpleSetupPtr simple_setup_;

  /** @brief The tesseract kinematics object */
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;

  /** @brief The mapping of environment links to kinematics links */
  tesseract_environment::AdjacencyMap::ConstPtr adj_map_;

  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_;
  tesseract_collision::ContinuousContactManager::Ptr continuous_contact_manager_;
};

class OMPLFreespacePlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  OMPLFreespacePlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 1,
    SolutionFound = 0,
    ErrorIsNotConfigured = -1,
    ErrorFailedToParseConfig = -2,
    ErrorFailedToFindValidSolution = -3
  };

private:
  std::string name_;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H
