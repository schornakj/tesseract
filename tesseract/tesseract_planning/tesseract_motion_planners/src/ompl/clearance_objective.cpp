#include <tesseract_motion_planners/ompl/clearance_objective.h>
#include <tesseract_environment/core/utils.h>

ompl::base::OptimizationObjectivePtr tesseract_motion_planners::getBalancedObjective(const ompl::base::SpaceInformationPtr& si,
                                                                                     tesseract_environment::Environment::ConstPtr env,
                                                                                     tesseract_kinematics::ForwardKinematics::ConstPtr kin)
{
    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::PathLengthOptimizationObjective(si));
    ompl::base::OptimizationObjectivePtr clearObj(new tesseract_motion_planners::ClearanceObjective(si, env, kin));
    ompl::base::MultiOptimizationObjective* opt = new ompl::base::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 10.0);
    opt->addObjective(clearObj, 1.0);
    return ompl::base::OptimizationObjectivePtr(opt);
}

tesseract_motion_planners::ClearanceObjective::ClearanceObjective(const ompl::base::SpaceInformationPtr& si,
                                                                  tesseract_environment::Environment::ConstPtr env,
                                                                  tesseract_kinematics::ForwardKinematics::ConstPtr kin)
  : ompl::base::StateCostIntegralObjective(si, true)
  , env_(std::move(env))
  , kin_(std::move(kin))
{
//  joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  joint_names_ = kin_->getJointNames();

  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  tesseract_environment::AdjacencyMap adj_map(env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->transforms);
  links_ = adj_map.getActiveLinkNames();

  dcm_ = env_->getDiscreteContactManager();
//  dcm_->setContactDistanceThreshold(1.0);
}

ompl::base::Cost tesseract_motion_planners::ClearanceObjective::stateCost(const ompl::base::State* s) const
{
//    const ompl::base::StateSpace& state_space = *si_->getStateSpace();

    const auto* state = s->as<ompl::base::RealVectorStateSpace::StateType>();

    const auto dof = si_->getStateDimension();
    Eigen::Map<Eigen::VectorXd> joint_angles(state->values, dof);

    tesseract_environment::EnvState::Ptr env_state = env_->getStateSolver()->getState(joint_names_, joint_angles);

    std::vector<tesseract_collision::ContactResultMap> results;
    bool in_contact = tesseract_environment::checkTrajectoryState(results, *dcm_, env_state, tesseract_collision::ContactTestType::ALL, false);

    if (results.size() == 0) return ompl::base::Cost(0.0);  // return zero cost if so far away from collision that zero contacts are reported

    double clearance = static_cast<double>(results.begin()->begin()->second[0].distance);

    double clearance_clamped = std::max(clearance, std::numeric_limits<double>::min());  // enforce that collision clearance should be nonzero and positive to calculate a valid cost

//    double cost = 1.0;

    double cost = 1 / clearance_clamped;

    std::cout << "Clearance: " << clearance << " Cost: " << cost << std::endl;

    return ompl::base::Cost(cost);
}
