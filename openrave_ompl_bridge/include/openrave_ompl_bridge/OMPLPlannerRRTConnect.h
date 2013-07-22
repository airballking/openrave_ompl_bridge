#ifndef OPENRAVE_OMPL_BRIDGE_OMPL_PLANNER_RTT_CONNECT_H
#define OPENRAVE_OMPL_BRIDGE_OMPL_PLANNER_RTT_CONNECT_H

#include <openrave-core.h>
#include <openrave/planner.h>

#include <openrave_ompl_bridge/OMPLPlannerParametersRRTConnect.h>

namespace openrave_ompl_bridge
{
  // plugin exposing the RRTConnect motion planner from the OMPL library to OpenRAVE
  class OMPLPlannerRRTConnect: public OpenRAVE::PlannerBase
  {
    public:
      // constructor and destructor required to declare and implement for planner plugin in OpenRAVE
      OMPLPlannerRRTConnect(OpenRAVE::EnvironmentBasePtr penv);
      virtual ~OMPLPlannerRRTConnect();

      // two ways of initializing a planning problem: (1) a parameters-object (2) parameters in xml-stream
      virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params);
      virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);

      // call to trigger planning of current problem
      virtual OpenRAVE::PlannerStatus PlanPath (OpenRAVE::TrajectoryBasePtr ptraj);

      // get for the current specification of the planning problem
      virtual PlannerParametersConstPtr GetParameters () const { return parameters_; }

    private:
      // internal pointer to the parameters of the planning problem
      OMPLPlannerParametersRRTConnectPtr parameters_;
  };

  // convenience typedef for shared pointers to objects of this class
  typedef boost::shared_ptr<OMPLPlannerRRTConnect> OMPLPlannerRRTConnectPtr;
} //namespace openrave_ompl_bridge

#endif //OPENRAVE_OMPL_BRIDGE_OMPL_PLANNER_RTT_CONNECT_H
