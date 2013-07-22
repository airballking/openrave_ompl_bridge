#include <openrave_ompl_bridge/OMPLPlannerRRTConnect.h>

using namespace OpenRAVE;

namespace openrave_ompl_bridge
{
  OMPLPlannerRRTConnect::OMPLPlannerRRTConnect(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::PlannerBase(penv)
  {
  }

  OMPLPlannerRRTConnect::~OMPLPlannerRRTConnect()
  {
  }

  bool OMPLPlannerRRTConnect::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
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

  OpenRAVE::PlannerBase::PlannerParametersConstPtr OMPLPlannerRRTConnect::GetParameters () const
  {
    return parameters_;
  }

} /* namespace openrave_ompl_bridge */
