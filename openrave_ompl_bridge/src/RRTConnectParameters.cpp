#include <openrave_ompl_bridge/RRTConnectParameters.h>

namespace openrave_ompl_bridge
{
  RRTConnectParameters::RRTConnectParameters() 
  {
  }

  bool RRTConnectParameters::serialize(std::ostream& O) const
  {
    if (!PlannerParameters::serialize(O))
    {    
      return false;
    }
    return true;
  }

  OpenRAVE::BaseXMLReader::ProcessElement RRTConnectParameters::startElement(const std::string& name, const std::list<std::pair<std::string, std::string> >& atts)
  {
    switch (OpenRAVE::PlannerBase::PlannerParameters::startElement(name, atts))
    {
      case PE_Pass:
        break;
      case PE_Support:
        return PE_Support;
      case PE_Ignore:
        return PE_Ignore;
    }

    return OpenRAVE::BaseXMLReader::PE_Ignore;
  }

  bool RRTConnectParameters::endElement(const std::string& name)
  {
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

  double RRTConnectParameters::GetTimeLimit()
  {
    return timelimit;
  }
} /* namespace openrave_ompl_bridge */
