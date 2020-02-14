/**
 * @file ompl_persistent_planner.cpp
 * @brief Tesseract OMPL motion planner for PRM planners and similar.
 *
 * @author Joseph Schornak
 * @date February 12, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_persistent_planner.h>
#include <tesseract_motion_planners/ompl/conversions.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>

namespace tesseract_motion_planners
{
/** @brief Construct a basic planner */
OMPLPersistentPlanner::OMPLPersistentPlanner(std::string name)
  : MotionPlanner(std::move(name))
  , config_(nullptr)
  , status_category_(std::make_shared<const OMPLMotionPlannerStatusCategory>(name_))
{
}

bool OMPLPersistentPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

tesseract_common::StatusCode OMPLPersistentPlanner::solve(PlannerResponse& response, bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  std::cout << "Graph contains " << prm_planner_->milestoneCount() << " milestones and " << prm_planner_->edgeCount() << " edges." << std::endl;

  auto term_cond = ompl::base::plannerOrTerminationCondition(ompl::base::timedPlannerTerminationCondition(config_->planning_time),
                                                             ompl::base::exactSolnPlannerTerminationCondition(config_->simple_setup->getProblemDefinition()));

  ompl::base::PlannerStatus status = prm_planner_->solve(term_cond);

  if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
  {
    response.status =
        tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorFailedToFindValidSolution, status_category_);
    return response.status;
  }

  if (config_->simplify)
  {
    config_->simple_setup->simplifySolution();
  }
  else
  {
    // Interpolate the path if it shouldn't be simplified and there are currently fewer states than requested
    auto num_output_states = static_cast<unsigned>(config_->n_output_states);
    if (config_->simple_setup->getSolutionPath().getStateCount() < num_output_states)
    {
      config_->simple_setup->getSolutionPath().interpolate(num_output_states);
    }
    else
    {
      // Now try to simplify the trajectory to get it under the requested number of output states
      // The interpolate function only executes if the current number of states is less than the requested
      config_->simple_setup->simplifySolution();
      if (config_->simple_setup->getSolutionPath().getStateCount() < num_output_states)
        config_->simple_setup->getSolutionPath().interpolate(num_output_states);
    }
  }

  tesseract_common::TrajArray traj = toTrajArray(config_->simple_setup->getSolutionPath());

  // Check and report collisions
  std::vector<tesseract_collision::ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = config_->tesseract->getEnvironmentConst()->getStateSolver();
  continuous_contact_manager_->setContactDistanceThreshold(0);
  collisions.clear();
  bool found = tesseract_environment::checkTrajectory(collisions,
                                                      *continuous_contact_manager_,
                                                      *state_solver,
                                                      kin_->getJointNames(),
                                                      traj,
                                                      config_->longest_valid_segment_length,
                                                      tesseract_collision::ContactTestType::FIRST,
                                                      verbose);

  // Set the contact distance back to original incase solve was called again.
  continuous_contact_manager_->setContactDistanceThreshold(config_->collision_safety_margin);

  // Send response
  response.joint_trajectory.trajectory = traj;
  response.joint_trajectory.joint_names = kin_->getJointNames();
  if (found)
  {
    response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision,
                                                   status_category_);
  }
  else
  {
    response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::SolutionFound, status_category_);
    CONSOLE_BRIDGE_logInform("%s, final trajectory is collision free", name_.c_str());
  }

  return response.status;
}

void OMPLPersistentPlanner::clear()
{
  request_ = PlannerRequest();
  config_ = nullptr;
  kin_ = nullptr;
  continuous_contact_manager_ = nullptr;
//  prm_planner_ = nullptr;
}

tesseract_common::StatusCode OMPLPersistentPlanner::isConfigured() const
{
  if (config_ == nullptr || kin_ == nullptr || continuous_contact_manager_ == nullptr || prm_planner_ == nullptr)
    return tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorIsNotConfigured, status_category_);

  return tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::IsConfigured, status_category_);
}

bool OMPLPersistentPlanner::updateConfiguration(tesseract_motion_planners::Waypoint::Ptr start, tesseract_motion_planners::Waypoint::Ptr end)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return false;
  }

//  config_ = std::move(config);

  if (!config_->updateGoalStates(start, end))
  {
    config_ = nullptr;
    return false;
  }

  prm_planner_->clearQuery();
  prm_planner_->setProblemDefinition(config_->simple_setup->getProblemDefinition());
//  prm_planner_->setup();

  return true;
}

bool OMPLPersistentPlanner::setConfiguration(OMPLPlannerConfig::Ptr config)
{
  // Reset state
  clear();

  config_ = std::move(config);
  if (!config_->generate())
  {
    config_ = nullptr;
    return false;
  }

  kin_ = config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator);
  const tesseract_environment::Environment::ConstPtr& env = config_->tesseract->getEnvironmentConst();
  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin_->getActiveLinkNames(), env->getCurrentState()->transforms);

  continuous_contact_manager_ = env->getContinuousContactManager();
  continuous_contact_manager_->setActiveCollisionObjects(adj_map->getActiveLinkNames());
  continuous_contact_manager_->setContactDistanceThreshold(config_->collision_safety_margin);

  prm_planner_ = std::make_shared<ompl::geometric::PRM>(config_->simple_setup->getSpaceInformation());
  prm_planner_->setProblemDefinition(config_->simple_setup->getProblemDefinition());

//  config_->simple_setup->setPlanner(prm_planner_);
  prm_planner_->setup();

  return true;
}

}  // namespace tesseract_motion_planners