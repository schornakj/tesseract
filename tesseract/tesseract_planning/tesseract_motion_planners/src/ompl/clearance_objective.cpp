#include <tesseract_motion_planners/ompl/clearance_objective.h>
#include <tesseract_environment/core/utils.h>

ompl::base::OptimizationObjectivePtr tesseract_motion_planners::getBalancedObjective(const ompl::base::SpaceInformationPtr& si,
                                                                                     tesseract_environment::Environment::ConstPtr env,
                                                                                     tesseract_kinematics::ForwardKinematics::ConstPtr kin)
{
    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::PathLengthOptimizationObjective(si));
    ompl::base::OptimizationObjectivePtr clearObj(new tesseract_motion_planners::ClearanceObjective(si, env, kin));
    ompl::base::MultiOptimizationObjective* opt = new ompl::base::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 5.0);
    opt->addObjective(clearObj, 5.0);
    return ompl::base::OptimizationObjectivePtr(opt);
}

tesseract_motion_planners::ClearanceObjective::ClearanceObjective(const ompl::base::SpaceInformationPtr& si,
                                                                  tesseract_environment::Environment::ConstPtr env,
                                                                  tesseract_kinematics::ForwardKinematics::ConstPtr kin)
  : ompl::base::StateCostIntegralObjective(si, true)
  , env_(std::move(env))
  , kin_(std::move(kin))
{
  joint_names_ = kin_->getJointNames();

  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  tesseract_environment::AdjacencyMap adj_map(env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->transforms);
  links_ = adj_map.getActiveLinkNames();

  state_solver_ = env_->getStateSolver();

  ccm_ = env_->getContinuousContactManager();
  ccm_->setContactDistanceThreshold(0.2);
  ccm_->setActiveCollisionObjects(links_);
}


ompl::base::Cost tesseract_motion_planners::ClearanceObjective::motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  const auto* start = s1->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto* finish = s2->as<ompl::base::RealVectorStateSpace::StateType>();

  // Get the contact manager and state solver for this thread.
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::ContinuousContactManager::Ptr cm;
  tesseract_environment::StateSolver::Ptr ss;
  mutex_.lock();
  auto it = continuous_contact_managers_.find(hash);
  if (it == continuous_contact_managers_.end())
  {
    cm = ccm_->clone();
    continuous_contact_managers_[hash] = cm;

    ss = state_solver_->clone();
    state_solver_managers_[hash] = ss;
  }
  else
  {
    cm = it->second;
    ss = state_solver_managers_[hash];
  }
  mutex_.unlock();

  const auto dof = si_->getStateDimension();
  Eigen::Map<Eigen::VectorXd> start_joints(start->values, dof);
  Eigen::Map<Eigen::VectorXd> finish_joints(finish->values, dof);

  tesseract_environment::EnvState::Ptr state0 = ss->getState(joint_names_, start_joints);
  tesseract_environment::EnvState::Ptr state1 = ss->getState(joint_names_, finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::CLOSEST);

  std::pair<std::string, std::string> pair = std::make_pair("aircraft", "eoat_link");  // TODO: avoid hardcoding

  auto result_it = contact_map.find(pair);

  if (result_it == contact_map.end()) return ompl::base::Cost(0.0);  // return zero cost if so far away from collision that zero contacts are reported

  double clearance_clamped = std::max(static_cast<double>(result_it->second[0].distance), std::numeric_limits<double>::min());  // enforce that collision clearance should be nonzero and positive to calculate a valid cost

  double cost = 1 / clearance_clamped;

  return ompl::base::Cost(cost);
}

