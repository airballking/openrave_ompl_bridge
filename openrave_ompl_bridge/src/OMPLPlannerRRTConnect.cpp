#include <openrave_ompl_bridge/OMPLPlannerRRTConnect.h>

using namespace openrave_ompl_bridge;

OMPLPlannerRRTConnect::OMPLPlannerRRTConnect(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::PlannerBase(penv)
{
  // do nothing
}

OMPLPlannerRRTConnect::~OMPLPlannerRRTConnect()
{
  // do nothing
}

bool OMPLPlannerRRTConnect::InitPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params)
{
  return false;
}

bool OMPLPlannerRRTConnect::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input)
{
  return false;
}

OpenRAVE::PlannerStatus OMPLPlannerRRTConnect::PlanPath (OpenRAVE::TrajectoryBasePtr ptraj)
{
  return OpenRAVE::PS_Failed;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr OMPLPlannerRRTConnect::GetParameters() const
{
  return parameters_;
}
