#include <openrave_ompl_bridge/RRTConnect.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace OpenRAVE;

namespace openrave_ompl_bridge
{
  RRTConnect::RRTConnect(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::PlannerBase(penv), parameters_(new RRTConnectParameters()), state_space_(new ompl::base::RealVectorStateSpace(0))
  {
  }

  RRTConnect::~RRTConnect()
  {
  }

  bool RRTConnect::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
  {
    robot_ = RobotPtr(new Robot(robot));

    CopyParameters(params);

    ResetStateSpaceDimensions();

    ResetStateSpaceBoundaries();

    ResetSimpleSetup();

    return true;
  }

  bool RRTConnect::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input)
  {
    RRTConnectParameters* parameters = new RRTConnectParameters();
    input >> (*parameters);
    return InitPlan(robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr(parameters));
  }

  OpenRAVE::PlannerStatus RRTConnect::PlanPath (OpenRAVE::TrajectoryBasePtr ptraj)
  {
    assert(ptraj);
    assert(parameters_);

    if(!EnsureInitializedPlan())
      return OpenRAVE::PS_Failed;

    if(!SolveWithTimelimit(parameters_->GetTimeLimit()))
      return OpenRAVE::PS_Failed;

    if(!SmoothenPath())
      return OpenRAVE::PS_Failed;

    if(!CopyFinalPath(ptraj))
      return OpenRAVE::PS_Failed;

    return OpenRAVE::PS_HasSolution;
  }

  OpenRAVE::PlannerBase::PlannerParametersConstPtr RRTConnect::GetParameters () const
  {
    return parameters_;
  }

  void RRTConnect::CopyParameters(OpenRAVE::PlannerBase::PlannerParametersConstPtr parameters)
  {
    parameters_.reset(new RRTConnectParameters());
    parameters_->copy(parameters);
  }

  void RRTConnect::ResetStateSpaceDimensions()
  {
    state_space_ = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(robot_->getDOF()));
  }

  void RRTConnect::ResetStateSpaceBoundaries()
  {
    ompl::base::RealVectorBounds bounds(robot_->getDOF());
    std::vector<double> lower_limits = robot_->getLowerJointLimits();
    std::vector<double> upper_limits = robot_->getUpperJointLimits();


    assert(robot_->getDOF() == lower_limits.size());
    assert(robot_->getDOF() == upper_limits.size());

    bounds.low = lower_limits;
    bounds.high = upper_limits;

    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  }

  void RRTConnect::ResetSimpleSetup()
  {
    assert(state_space_);

    simple_setup_ = OMPLSimpleSetupPtr(new ompl::geometric::SimpleSetup(state_space_));
    simple_setup_->setStateValidityChecker(boost::bind(&openrave_ompl_bridge::RRTConnect::IsStateValid, this, _1));
    simple_setup_->setStartState(GetStartState());
    simple_setup_->setGoalState(GetGoalState());
  }

  ompl::base::ScopedState<> RRTConnect::GetStartState()
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

  ompl::base::ScopedState<> RRTConnect::GetGoalState()
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

  unsigned int RRTConnect::GetStateSpaceDimensions()
  {
    assert(state_space_);

    return state_space_->as<ompl::base::RealVectorStateSpace>()->getDimension();
  }

  bool RRTConnect::EnsureInitializedPlan()
  {
    if(!simple_setup_)
    {
      RAVELOG_ERROR("Internal pointer to simple setup was NULL. Aborting!.\n");
      return false;
    }

      return true;
  }

  bool RRTConnect::SolveWithTimelimit(double timelimit)
  {
    assert(simple_setup_);
    return simple_setup_->solve(timelimit);
  }

  bool RRTConnect::SmoothenPath(double timelimit)
  {
    if(!EnsureSolutionPath())
      return false;

    simple_setup_->simplifySolution(timelimit);
    return true;
  }

  bool RRTConnect::CopyFinalPath(OpenRAVE::TrajectoryBasePtr ptraj)
  {
    assert(ptraj);
    
    if(!EnsureSolutionPath())
      return false;

    InitSolutionPathContainer(ptraj);

    std::vector<ompl::base::State*> states = GetSolutionPath();
    for (unsigned int i=0; i<states.size(); i++)
    {
      ptraj->Insert(i, TransformPathPoint(states[i]).q, true);
    }
    
    return true;
  }

  void RRTConnect::InitSolutionPathContainer(OpenRAVE::TrajectoryBasePtr ptraj)
  {
    ptraj->Init(robot_->getConfigurationSpec());
  }

  bool RRTConnect::EnsureSolutionPath()
  {
    assert(simple_setup_);

    if(!simple_setup_->haveSolutionPath())
    {
      RAVELOG_ERROR("Now solution path was found. Aborting!\n");
      return false;
    }

    return true;
  }

  std::vector<ompl::base::State*> RRTConnect::GetSolutionPath()
  {
    assert(simple_setup_);

    return simple_setup_->getSolutionPath().getStates();
  }

  OpenRAVE::TrajectoryBase::Point RRTConnect::TransformPathPoint(ompl::base::State* state)
  {
    const ompl::base::RealVectorStateSpace::StateType* state_cast = state->as<ompl::base::RealVectorStateSpace::StateType>();

    assert(state_cast);
    assert(GetStateSpaceDimensions() == robot_->getDOF());

    OpenRAVE::TrajectoryBase::Point result;
    for(unsigned int j = 0; j < robot_->getDOF(); j++)
      result.q.push_back((*state_cast)[j]);

    return result;
  }

  bool RRTConnect::IsStateValid(const ompl::base::State* state)
  {
    assert(state);

    const ompl::base::RealVectorStateSpace::StateType* realVectorState = state->as<ompl::base::RealVectorStateSpace::StateType>();

    assert(realVectorState);
    
    std::vector<double> values;
    for(unsigned int i = 0; i < robot_->getDOF(); i++)
    {
      values.push_back(realVectorState->values[i]);
    }

    return IsActiveRobotConfigurationInCollision(values);
  }

  bool RRTConnect::IsActiveRobotConfigurationInCollision(std::vector<double>& joint_values)
  {
    assert(robot_);

    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    robot_->setJointValues(joint_values);
    return (GetEnv()->CheckCollision(KinBodyConstPtr(robot_->getRobotPointer())) || robot_->isInSelfCollision());
  }
} /* namespace openrave_ompl_bridge */
