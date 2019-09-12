/**
 * @file discrete_motion_validator.h
 * @brief Tesseract OMPL planner discrete collision check between two states
 *
 * @author Jonathan Meyer, Levi Armstrong
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
#ifndef TESSERACT_MOTION_PLANNERS_DISCRETE_MOTION_VALIDATOR_H
#define TESSERACT_MOTION_PLANNERS_DISCRETE_MOTION_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/MotionValidator.h>
#include <mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_motion_planners
{
/** @brief Continuous collision check between two states */
class DiscreteMotionValidator : public ompl::base::MotionValidator
{
public:
  DiscreteMotionValidator(ompl::base::SpaceInformationPtr space_info,
                          tesseract_environment::Environment::ConstPtr env,
                          tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

  bool checkMotion(const ompl::base::State* s1,
                   const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& lastValid) const override;

private:
  bool discreteCollisionCheck(const ompl::base::State* s2) const;

  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
  std::vector<std::string> links_;
  std::vector<std::string> joints_;
  mutable std::mutex mutex_;
  mutable std::map<unsigned long int, tesseract_collision::DiscreteContactManager::Ptr> contact_managers_;
};
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DISCRETE_MOTION_VALIDATOR_H