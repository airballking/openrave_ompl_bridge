#ifndef OPENRAVE_OMPL_BRIDGE_RRT_CONNECT_PARAMETERS_H
#define OPENRAVE_OMPL_BRIDGE_RRT_CONNECT_PARAMETERS_H

#include <openrave-core.h>
#include <openrave/planner.h>

#include <ompl/geometric/SimpleSetup.h>

namespace openrave_ompl_bridge
{
  // a calls to hold the parameters for the RRTConnect algorithm in the OMPL library
  class RRTConnectParameters: public OpenRAVE::PlannerBase::PlannerParameters
  {
    public:
      // well, the default constructor of this class ;)
      RRTConnectParameters();
   
      // accessors to parameters of interest to others 
      std::vector<double> GetStartConfiguration();
      std::vector<double> GetGoalConfiguration();
      double GetTimeLimit();
      double GetSmoothingTimeLimit();

      // actual parameters...
      double timelimit;
      double smoothing_timelimit;
    protected:
      // helper function which generates an xml-representation of this object and flushes it into an output-stream
      virtual bool serialize(std::ostream& O, int options=0) const;

      // aux-function for xml-parsing, called at start of each xml-field, returns PE_SUPPORT if the field will be read by class
      ProcessElement startElement(const std::string& name, const OpenRAVE::AttributesList& atts); 

      // another aux-function for xml-parsing, called at end of each xml-field, returns true if parsing of field was successful
      virtual bool endElement(const std::string& name);
     
      // internal parameter
      bool is_processing;
  };

  // useful typedef for pointers to objects of this class
  typedef boost::shared_ptr<RRTConnectParameters> RRTConnectParametersPtr;
} //namespace openrave_ompl_bridge

#endif // OPENRAVE_OMPL_BRIDGE_RRT_CONNECT_PARAMTERS_H
