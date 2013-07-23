#include <openrave_ompl_bridge/OMPLPlannerRRTConnect.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace OpenRAVE;

namespace openrave_ompl_bridge
{
  OMPLPlannerRRTConnect::OMPLPlannerRRTConnect(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::PlannerBase(penv), 
      state_space_(new ompl::base::RealVectorStateSpace(0))
  {
  }

  OMPLPlannerRRTConnect::~OMPLPlannerRRTConnect()
  {
  }

  bool OMPLPlannerRRTConnect::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
  {
    if(!CopyRobot(robot))
      return false;

    if(!CopyParameters(params))
      return false;

    if(!ResetStateSpaceDimensions())
      return false;

    if(!ResetStateSpaceBoundaries())
      return false;

    ResetSimpleSetup();

    return true;
  }

  bool OMPLPlannerRRTConnect::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input)
  {
    OMPLPlannerParametersRRTConnect* parameters = new OMPLPlannerParametersRRTConnect();
    input >> (*parameters);
    return InitPlan(robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr(parameters));
  }

  OpenRAVE::PlannerStatus OMPLPlannerRRTConnect::PlanPath (OpenRAVE::TrajectoryBasePtr ptraj)
  {
    // TODO(Georg): implement me
    return OpenRAVE::PS_Failed;
  }

  OpenRAVE::PlannerBase::PlannerParametersConstPtr OMPLPlannerRRTConnect::GetParameters () const
  {
    return parameters_;
  }

  bool OMPLPlannerRRTConnect::CopyRobot(OpenRAVE::RobotBasePtr robot)
  {
    if(!robot)
    {
      RAVELOG_ERROR("Passed pointer to robot was NULL. Aborting!\n");
      return false;
    }
    
    robot_ = robot;
    return true;
  }

  bool OMPLPlannerRRTConnect::CopyParameters(OpenRAVE::PlannerBase::PlannerParametersConstPtr parameters)
  {
    if(!parameters)
    {
      RAVELOG_ERROR("Passed pointer to parameters was NULL. Aborting!\n");
      return false;
    }

    parameters_->copy(parameters);
    return true;
  }

  bool OMPLPlannerRRTConnect::ResetStateSpaceDimensions()
  {
    if(!EnsureActiveRobotDOF())
      return false;
    
    state_space_ = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(GetRobotDOF()));

    return true; 
  }

  bool OMPLPlannerRRTConnect::ResetStateSpaceBoundaries()
  {
    if(!EnsureActiveRobotDOF())
      return false;
   
    ompl::base::RealVectorBounds bounds(GetRobotDOF());
    std::vector<double> lower_limits, upper_limits;
    GetRobotActiveJointLimits(lower_limits, upper_limits);     

    assert(GetRobotDOF() == lower_limits.size());
    assert(GetRobotDOF() == upper_limits.size());

    for (unsigned int i=0; i<GetRobotDOF(); i++)
    {
      bounds.setLow(i, lower_limits[i]);
      bounds.setHigh(i, upper_limits[i]);
    }

    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    return true; 
  }

  void OMPLPlannerRRTConnect::ResetSimpleSetup()
  {
    assert(state_space_);

    simple_setup_ = OMPLSimpleSetupPtr(new ompl::geometric::SimpleSetup(state_space_));
    simple_setup_->setStateValidityChecker(boost::bind(&openrave_ompl_bridge::OMPLPlannerRRTConnect::IsStateValid, this, _1));
    simple_setup_->setStartState(GetStartState());
    simple_setup_->setGoalState(GetGoalState());
  }

  ompl::base::ScopedState<> OMPLPlannerRRTConnect::GetStartState()
  {
    assert(state_space_);
    assert(parameters_);
   
    std::vector<double> start_config = parameters_->GetStartConfiguration();
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(state_space_);

    assert(start_config.size() == GetStateSpaceDimensions());

    for (unsigned int i=0; i<GetStateSpaceDimensions(); i++)
    {
      start->values[i] = start_config[i];
    }
    
    return start;
  }

  ompl::base::ScopedState<> OMPLPlannerRRTConnect::GetGoalState()
  {
    assert(state_space_);
    assert(parameters_);
   
    std::vector<double> goal_config = parameters_->GetGoalConfiguration();
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(state_space_);

    assert(goal_config.size() == GetStateSpaceDimensions());

    for (unsigned int i=0; i<GetStateSpaceDimensions(); i++)
    {
      goal->values[i] = goal_config[i];
    }
    
    return goal;
  }

  unsigned int OMPLPlannerRRTConnect::GetStateSpaceDimensions()
  {
    assert(state_space_);

    return state_space_->as<ompl::base::RealVectorStateSpace>()->getDimension();
  }

  bool OMPLPlannerRRTConnect::IsStateValid(const ompl::base::State* state)
  {
    assert(state);

    const ompl::base::RealVectorStateSpace::StateType* realVectorState = state->as<ompl::base::RealVectorStateSpace::StateType>();

    assert(realVectorState);
    
    std::vector<double> values;
    for(unsigned int i = 0; i < GetRobotDOF(); i++)
    {
      values.push_back(realVectorState->values[i]);
    }

    return CheckForRobotCollisions(values);
  }

  unsigned int OMPLPlannerRRTConnect::GetRobotDOF()
  {
    assert(robot_);
    return robot_->GetActiveDOF();
  }

  bool OMPLPlannerRRTConnect::EnsureActiveRobotDOF()
  {
    if(GetRobotDOF())
    {
      RAVELOG_ERROR("Given robot does not have any active joints. Aborting init!\n");
      return false;
    }

    return true;
  }

  void OMPLPlannerRRTConnect::GetRobotActiveJointLimits(std::vector<double>& lower, std::vector<double>& upper)
  {
    assert(robot_);
    robot_->GetActiveDOFLimits(lower, upper);
  }

  bool OMPLPlannerRRTConnect::CheckForRobotCollisions(std::vector<double>& joint_values)
  {
    assert(robot_);
    assert(joint_values.size() == GetRobotDOF());

    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    robot_->SetActiveDOFValues(joint_values);
    return GetEnv()->CheckCollision(KinBodyConstPtr(robot_)) || robot_->CheckSelfCollision();
  }
} /* namespace openrave_ompl_bridge */

