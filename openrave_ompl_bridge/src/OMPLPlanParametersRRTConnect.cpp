#include <openrave_ompl_bridge/OMPLPlanParametersRRTConnect.h>

using namespace openrave_ompl_bridge;

OMPLPlannerParametersRRTConnect::OMPLPlannerParametersRRTConnect()
{
  // do nothing
}

bool OMPLPlannerParametersRRTConnect::serialize(std::ostream& O) const
{
  return false;
}

OpenRAVE::BaseXMLReader::ProcessElement OMPLPlannerParametersRRTConnect::startElement(const std::string& name, const std::list<std::pair<std::string, std::string> >& atts)
{
  return BaseXMLReader::PE_Pass;
}

bool OMPLPlannerParametersRRTConnect::endElement(const std::string& name)
{
  return false;
}
