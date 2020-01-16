#ifndef TESSERACT_MOTION_PLANNERS_CLEARANCE_OBJECTIVE_H
#define TESSERACT_MOTION_PLANNERS_CLEARANCE_OBJECTIVE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_collision/core/discrete_contact_manager.h>

namespace tesseract_motion_planners
{

ompl::base::OptimizationObjectivePtr getBalancedObjective(const ompl::base::SpaceInformationPtr& si,
                                                          tesseract_environment::Environment::ConstPtr env,
                                                          tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  /** @brief Optimization objective that maximizes clearance between the robot and the environment */
class ClearanceObjective : public ompl::base::StateCostIntegralObjective
{
public:
  ClearanceObjective(const ompl::base::SpaceInformationPtr& si,
                     tesseract_environment::Environment::ConstPtr env,
                     tesseract_kinematics::ForwardKinematics::ConstPtr kin
                     );

  ompl::base::Cost stateCost(const ompl::base::State* s) const override;

private:
  /** @brief The Tesseract Environment */
  tesseract_environment::Environment::ConstPtr env_;

  /** @brief The Tesseract Forward Kinematics */
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;

  /** @brief A list of active links */
  std::vector<std::string> links_;

  tesseract_collision::DiscreteContactManager::Ptr dcm_;

  /** @brief A list of active joints */
  std::vector<std::string> joint_names_;
};
}
#endif // TESSERACT_MOTION_PLANNERS_CLEARANCE_OBJECTIVE_H
