#ifndef TESSERACT_MOTION_PLANNERS_CLEARANCE_OBJECTIVE_H
#define TESSERACT_MOTION_PLANNERS_CLEARANCE_OBJECTIVE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/state_solver.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

namespace tesseract_motion_planners
{
/** @brief Create an optimization objective balanced between minimizing joint motion and staying away from collision objects.**/
ompl::base::OptimizationObjectivePtr getBalancedObjective(const ompl::base::SpaceInformationPtr& si,
                                                          tesseract_environment::Environment::ConstPtr env,
                                                          tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  /** @brief Optimization objective that maximizes clearance between the robot and the environment */
class ClearanceObjective : public ompl::base::StateCostIntegralObjective
{
public:
  ClearanceObjective(const ompl::base::SpaceInformationPtr& si,
                     tesseract_environment::Environment::ConstPtr env,
                     tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  /** @brief Calculate a cost for motion between two states. The cost is based on the reciprocal of the closest distance between the EOAT and the environment. */
  ompl::base::Cost motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const override;

private:
  /** @brief The Tesseract Environment */
  tesseract_environment::Environment::ConstPtr env_;

  /** @brief The Tesseract Forward Kinematics */
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;

  /**< @brief The tesseract state solver */
  tesseract_environment::StateSolver::ConstPtr state_solver_;

  /** @brief A list of active links */
  std::vector<std::string> links_;

  /** @brief The Tesseract continuous contact manager */
  tesseract_collision::ContinuousContactManager::Ptr ccm_;

  /** @brief A list of active joints */
  std::vector<std::string> joint_names_;

  /** @brief Contact manager caching mutex */
  mutable std::mutex mutex_;

  /** @brief The continuous contact manager cache */
  mutable std::map<unsigned long int, tesseract_collision::ContinuousContactManager::Ptr> continuous_contact_managers_;

  /** @brief The state solver manager cache */
  mutable std::map<unsigned long int, tesseract_environment::StateSolver::Ptr> state_solver_managers_;

};
}
#endif // TESSERACT_MOTION_PLANNERS_CLEARANCE_OBJECTIVE_H
