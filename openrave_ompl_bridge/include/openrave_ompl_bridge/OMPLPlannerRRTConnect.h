#ifndef OPENRAVE_OMPL_BRIDGE_OMPL_PLANNER_RTT_CONNECT_H
#define OPENRAVE_OMPL_BRIDGE_OMPL_PLANNER_RTT_CONNECT_H

// basic openrave-stuff to wrap us in an OpenRAVE planner plugin
#include <openrave-core.h>
#include <openrave/planner.h>

// our parameter object
#include <openrave_ompl_bridge/OMPLPlannerParametersRRTConnect.h>

// parts of OMPL we're using
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>

// some standard C-stuff
#include <vector>

namespace openrave_ompl_bridge
{
  // convenience definition because we wanna have simple-setup-pointer as a member
  typedef boost::shared_ptr<ompl::geometric::SimpleSetup> OMPLSimpleSetupPtr;

  // plugin exposing the RRTConnect motion planner from the OMPL library to OpenRAVE
  class OMPLPlannerRRTConnect: public OpenRAVE::PlannerBase
  {
    public:
      // constructor and destructor required to declare and implement for planner plugin in OpenRAVE
      OMPLPlannerRRTConnect(OpenRAVE::EnvironmentBasePtr penv);
      virtual ~OMPLPlannerRRTConnect();

      // two ways of initializing a planning problem: (1) a parameters-object (2) parameters in xml-stream
      virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
      virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);

      // call to trigger planning of current problem
      virtual OpenRAVE::PlannerStatus PlanPath (OpenRAVE::TrajectoryBasePtr ptraj);

      // get for the current specification of the planning problem
      virtual OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters () const;

    private:
      // internal helper functions for init...
      bool CopyRobot(OpenRAVE::RobotBasePtr robot);
      bool CopyParameters(OpenRAVE::PlannerBase::PlannerParametersConstPtr parameters);
      bool ResetStateSpaceDimensions();
      bool ResetStateSpaceBoundaries();
      void ResetSimpleSetup();
      ompl::base::ScopedState<> GetStartState();
      ompl::base::ScopedState<> GetGoalState();
      unsigned int GetStateSpaceDimensions();

      // helper functions required by OMPL
      bool IsStateValid(const ompl::base::State* state);

      // internal robot helper functions...
      unsigned int GetRobotDOF();
      bool EnsureActiveRobotDOF();
      void GetRobotActiveJointLimits(std::vector<double>& lower, std::vector<double>& upper);
      bool CheckForRobotCollisions(std::vector<double>& joint_values);
      
      // internal members...
      OMPLPlannerParametersRRTConnectPtr parameters_;
      OMPLSimpleSetupPtr simple_setup_;
      ompl::base::StateSpacePtr state_space_;
      OpenRAVE::RobotBasePtr robot_;
  };

  // convenience typedef for shared pointers to objects of this class
  typedef boost::shared_ptr<OMPLPlannerRRTConnect> OMPLPlannerRRTConnectPtr;
} //namespace openrave_ompl_bridge

#endif //OPENRAVE_OMPL_BRIDGE_OMPL_PLANNER_RTT_CONNECT_H
