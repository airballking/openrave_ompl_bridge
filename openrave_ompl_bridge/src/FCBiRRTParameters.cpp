#include <openrave_ompl_bridge/FCBiRRTParameters.h>
#include <openrave_ompl_bridge/YamlUtils.h>

namespace openrave_ompl_bridge
{
  FCBiRRTParameters::FCBiRRTParameters() : RRTConnectParameters::RRTConnectParameters(), is_processing(false)
  {
    _vXMLParameters.push_back("goal_config");
    _vXMLParameters.push_back("goal_command");
    _vXMLParameters.push_back("path_config");
    _vXMLParameters.push_back("path_command");
  }

  bool FCBiRRTParameters::serialize(std::ostream& O, int options) const
  {
    if( !RRTConnectParameters::serialize(O, options&~1) )
    {
      return false;
    }

    YAML::Emitter goal_config_out, path_config_out, goal_command_out, path_command_out;

    goal_config_out << goal_config;
    path_config_out << path_config;
    goal_command_out << goal_command;
    path_command_out << path_command;

    O << "<goal_config>" << goal_config_out.c_str() << "</goal_config>" << std::endl;
    O << "<path_config>" << path_config_out.c_str() << "</path_config>" << std::endl;
    O << "<goal_command>" << goal_command_out.c_str() << "</goal_command>" << std::endl;
    O << "<path_command>" << path_command_out.c_str() << "</path_command>" << std::endl;

    if( !(options & 1) ) 
    {
      O << _sExtraParameters << std::endl;
    }
 
    return !!O;
  }

  OpenRAVE::BaseXMLReader::ProcessElement FCBiRRTParameters::startElement(const std::string& name, const OpenRAVE::AttributesList& atts)
  {
    if( is_processing ) 
    {
      return PE_Ignore;
    }

    switch( RRTConnectParameters::startElement(name,atts) ) 
    {
      case PE_Pass: break;
      case PE_Support: return PE_Support;
      case PE_Ignore: return PE_Ignore;
    }
 
    is_processing = name=="goal_config" || "path_config" || "goal_command" || "path_command";

    return is_processing ? PE_Support : PE_Pass;
  }

  bool FCBiRRTParameters::endElement(const std::string& name)
  {
    if( is_processing ) 
    {
      if(name == "goal_config") 
      {
        YAML::Parser parser(_ss);
        YAML::Node doc;
        while(parser.GetNextDocument(doc))
          doc >> goal_config;
      }
      else if(name == "path_config") 
      {
        YAML::Parser parser(_ss);
        YAML::Node doc;
        while(parser.GetNextDocument(doc))
          doc >> path_config;
      }
      else if(name == "goal_command") 
      {
        YAML::Parser parser(_ss);
        YAML::Node doc;
        while(parser.GetNextDocument(doc))
          doc >> goal_command;
      }
      else if(name == "path_command") 
      {
        YAML::Parser parser(_ss);
        YAML::Node doc;
        while(parser.GetNextDocument(doc))
          doc >> path_command;
      }
      else
      {
        RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
      }
    is_processing = false;
    return false;
    }
 
    // give a chance for the parent parameters to get processed
    return RRTConnectParameters::endElement(name);
  }

  const std::vector<Constraint>& FCBiRRTParameters::GetGoalConfig()
  {
    return goal_config; 
  }

  const std::vector<Constraint>& FCBiRRTParameters::GetPathConfig()
  { 
    return path_config; 
  }

  const Ranges& FCBiRRTParameters::GetGoalCommand()
  { 
    return goal_command; 
  }

  const Ranges& FCBiRRTParameters::GetPathCommand()
  { 
    return path_command; 
  }

} /* namespace openrave_ompl_bridge */
