#ifndef OPENRAVE_OMPL_BRIDGE_CONNECT_H
#define OPENRAVE_OMPL_BRIDGE_RTT_CONNECT_H

// basic openrave-stuff to wrap us in an OpenRAVE planner plugin
#include <openrave-core.h>
#include <openrave/planner.h>

// our parameter object
#include <openrave_ompl_bridge/RRTConnectParameters.h>

// a convenience wrapper around OpenRAVE's RobotBase
#include <openrave_ompl_bridge/Robot.h>

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
  class RRTConnect: public OpenRAVE::PlannerBase
  {
    public:
      /* THE STANDARD OPENRAVE PLANNER PLUGIN INTERFACE */
      // constructor and destructor required to declare and implement for planner plugin in OpenRAVE
      RRTConnect(OpenRAVE::EnvironmentBasePtr penv);
      virtual ~RRTConnect();

      // two ways of initializing a planning problem: (1) a parameters-object (2) parameters in xml-stream
      virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
      virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);

      // call to trigger planning of current problem
      virtual OpenRAVE::PlannerStatus PlanPath (OpenRAVE::TrajectoryBasePtr ptraj);

      // getter for the current specification of the planning problem
      virtual OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters () const;

    private:
      /* OMPL- AND RRTCONNECT-SPECIFIC CODE */
      // internal helper functions for init...
      void CopyParameters(OpenRAVE::PlannerBase::PlannerParametersConstPtr parameters);
      void ResetStateSpace();
      void ResetSimpleSetup();
      ompl::base::ScopedState<> TransformState(const std::vector<double>& state);
      unsigned int GetStateSpaceDimensions();

      //internal helper functions for PlanPath...
      void CopyFinalPath(OpenRAVE::TrajectoryBasePtr ptraj);
      OpenRAVE::TrajectoryBase::Point TransformPathPoint(ompl::base::State* state);

      // helper functions required by OMPL
      bool IsStateValid(const ompl::base::State* state);
      bool IsActiveRobotConfigurationInCollision(std::vector<double>& joint_values);

      // internal members...
      RRTConnectParametersPtr parameters_;
      RobotPtr robot_;
      OMPLSimpleSetupPtr simple_setup_;
      ompl::base::StateSpacePtr state_space_;
  };

  // convenience typedef for shared pointers to objects of this class
  typedef boost::shared_ptr<RRTConnect> RRTConnectPtr;
} //namespace openrave_ompl_bridge

#endif //OPENRAVE_OMPL_BRIDGE_RTT_CONNECT_H
