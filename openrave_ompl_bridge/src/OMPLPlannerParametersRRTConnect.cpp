#include <openrave_ompl_bridge/OMPLPlannerParametersRRTConnect.h>

namespace openrave_ompl_bridge
{
  OMPLPlannerParametersRRTConnect::OMPLPlannerParametersRRTConnect() 
  {
  }

  bool OMPLPlannerParametersRRTConnect::serialize(std::ostream& O) const
  {
    return false;
  }

  OpenRAVE::BaseXMLReader::ProcessElement OMPLPlannerParametersRRTConnect::startElement(const std::string& name, const std::list<std::pair<std::string, std::string> >& atts)
  {
    return OpenRAVE::BaseXMLReader::PE_Ignore;
  }

  bool OMPLPlannerParametersRRTConnect::endElement(const std::string& name)
  {
    return false;
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
