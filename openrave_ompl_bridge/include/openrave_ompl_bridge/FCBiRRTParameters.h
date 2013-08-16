#ifndef OPENRAVE_OMPL_BRIDGE_FCBIRRT_PARAMETERS_H
#define OPENRAVE_OMPL_BRIDGE_FCBIRRT_PARAMETERS_H

#include <openrave_ompl_bridge/RRTConnectParameters.h>
#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Controller.h>
#include <vector>

namespace openrave_ompl_bridge
{
  // a calls to hold the parameters for the RRTConnect algorithm in the OMPL library
  class FCBiRRTParameters: public RRTConnectParameters
  {
    public:
      // well, the default constructor of this class ;)
      FCBiRRTParameters();

      // constraints for path and goal 
      std::vector<Constraint> goal_config, path_config;
      Ranges goal_command, path_command;

      // some getters 
      const std::vector<Constraint>& GetGoalConfig();
      const std::vector<Constraint>& GetPathConfig();
      const Ranges& GetGoalCommand();
      const Ranges& GetPathCommand();

    protected:
      // helper function which generates an xml-representation of this object and flushes it into an output-stream
      virtual bool serialize(std::ostream& O, int options=0) const;
 
      // aux-function for xml-parsing, called at start of each xml-field, returns PE_SUPPORT if the field will be read by class
      virtual ProcessElement startElement(const std::string& name, const OpenRAVE::AttributesList& atts); 
 
      // another aux-function for xml-parsing, called at end of each xml-field, returns true if parsing of field was successful
      virtual bool endElement(const std::string& name);
   
    private:
      bool is_processing;
  };

  // useful typedef for pointers to objects of this class
  typedef boost::shared_ptr<FCBiRRTParameters> FCBiRRTParametersPtr;
} //namespace openrave_ompl_bridge

#endif // OPENRAVE_OMPL_BRIDGE_FCBIRRT_PARAMTERS_H
