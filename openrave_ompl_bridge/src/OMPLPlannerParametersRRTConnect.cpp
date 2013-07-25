#include <openrave_ompl_bridge/OMPLPlannerParametersRRTConnect.h>

namespace openrave_ompl_bridge
{
  OMPLPlannerParametersRRTConnect::OMPLPlannerParametersRRTConnect() 
  {
  }

  bool OMPLPlannerParametersRRTConnect::serialize(std::ostream& O) const
  {
    if (!PlannerParameters::serialize(O))
    {    
      return false;
    }
    return true;
  }

  OpenRAVE::BaseXMLReader::ProcessElement OMPLPlannerParametersRRTConnect::startElement(const std::string& name, const std::list<std::pair<std::string, std::string> >& atts)
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

  bool OMPLPlannerParametersRRTConnect::endElement(const std::string& name)
  {
    return PlannerParameters::endElement(name);
  }

  std::vector<double> OMPLPlannerParametersRRTConnect::GetStartConfiguration()
  {
    return vinitialconfig;
  }

  std::vector<double> OMPLPlannerParametersRRTConnect::GetGoalConfiguration()
  {
    return vgoalconfig;
  }

  double OMPLPlannerParametersRRTConnect::GetTimeLimit()
  {
    return timelimit;
  }
} /* namespace openrave_ompl_bridge */
