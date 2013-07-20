#ifndef OPENRAVE_OMPL_BRIDGE_OMPL_PLAN_PARAMETERS_RTT_CONNECT_H
#define OPENRAVE_OMPL_BRIDGE_OMPL_PLAN_PARAMETERS_RTT_CONNECT_H

#include <openrave-core.h>
#include <openrave/planner.h>

namespace openrave_ompl_bridge
{
  // a calls to hold the parameters for the RRTConnect algorithm in the OMPL library
  class OMPLPlannerParametersRRTConnect: public OpenRAVE::PlannerBase::PlannerParameters
  {
    public:
      // well, the default constructor of this class ;)
      OMPLPlannerParametersRRTConnect();
    
      // all parameters we need to set up the RRTConnect from OMPL

    protected:
      // helper function which generates an xml-representation of this object and flushes it into an output-stream
      virtual bool serialize(std::ostream& O) const;

      // aux-function for xml-parsing, called at start of each xml-field, returns PE_SUPPORT if the field will be read by class
      ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string, std::string> >& atts);

      // another aux-function for xml-parsing, called at end of each xml-field, returns true if parsing of field was successful
      virtual bool endElement(const std::string& name);
  };

  // useful typedef for pointers to objects of this class
  typedef boost::shared_ptr<OMPLPlannerParametersRRTConnect> OMPLPlannerParametersRRTConnectPtr;
} //namespace openrave_ompl_bridge

#endif // OPENRAVE_OMPL_BRIDGE_OMPL_PLAN_PARAMETERS_RTT_CONNECT_H
