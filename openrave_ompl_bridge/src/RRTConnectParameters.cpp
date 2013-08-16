#include <openrave_ompl_bridge/RRTConnectParameters.h>

namespace openrave_ompl_bridge
{
  RRTConnectParameters::RRTConnectParameters() : is_processing(false) 
  {
    _vXMLParameters.push_back("planning_timelimit");
    _vXMLParameters.push_back("smoothing_timelimit");
  }

  bool RRTConnectParameters::serialize(std::ostream& O, int options) const
  {
    if( !OpenRAVE::PlannerBase::PlannerParameters::serialize(O, options&~1) )
    {
      return false;
    }

    O << "<planning_timelimit>" << planning_timelimit << "</planning_timelimit>" << std::endl;
    O << "<smoothing_timelimit>" << smoothing_timelimit << "</smoothing_timelimit>" << std::endl;

    if( !(options & 1) ) 
    {
      O << _sExtraParameters << std::endl;
    }
 
    return !!O;
  }
 
  OpenRAVE::BaseXMLReader::ProcessElement RRTConnectParameters::startElement(const std::string& name, const OpenRAVE::AttributesList& atts)
  {
    if( is_processing ) 
    {
    return PE_Ignore;
    }

    switch( OpenRAVE::PlannerBase::PlannerParameters::startElement(name,atts) ) 
    {
      case PE_Pass: break;
      case PE_Support: return PE_Support;
      case PE_Ignore: return PE_Ignore;
    }
 
    is_processing = name=="planning_timelimit" || name=="smoothing_timelimit";

    return is_processing ? PE_Support : PE_Pass;
  }
 
  bool RRTConnectParameters::endElement(const std::string& name)
  {
    if( is_processing ) 
    {
      if( name == "planning_timelimit") 
      {
        _ss >> planning_timelimit;
      }
      else if( name == "smoothing_timelimit") 
      {
        _ss >> smoothing_timelimit;
      }
      else
 
      {
        RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
      }
    is_processing = false;
    return false;
    }
 
    // give a chance for the default parameters to get processed
    return PlannerParameters::endElement(name);
  }

  std::vector<double> RRTConnectParameters::GetStartConfiguration()
  {
    return vinitialconfig;
  }

  std::vector<double> RRTConnectParameters::GetGoalConfiguration()
  {
    return vgoalconfig;
  }

  double RRTConnectParameters::GetPlanningTimeLimit()
  {
    return planning_timelimit;
  }

  double RRTConnectParameters::GetSmoothingTimeLimit()
  {
    return smoothing_timelimit;
  }
} /* namespace openrave_ompl_bridge */
