#include <openrave_ompl_bridge/FCBiRRTParameters.h>

namespace openrave_ompl_bridge
{
  FCBiRRTParameters::FCBiRRTParameters() : RRTConnectParameters::RRTConnectParameters(), is_processing(false)
  {

  }

  bool FCBiRRTParameters::serialize(std::ostream& O, int options) const
  {
    if( !RRTConnectParameters::serialize(O, options&~1) )
    {
      return false;
    }

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
 
    is_processing = false;

    return is_processing ? PE_Support : PE_Pass;
  }

  bool FCBiRRTParameters::endElement(const std::string& name)
  {
    if( is_processing ) 
    {/*
      if( name == "planning_timelimit") 
      {
        _ss >> planning_timelimit;
      }
      else if( name == "smoothing_timelimit") 
      {
        _ss >> smoothing_timelimit;
      }
      else*/
//      {
        RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
//      }
    is_processing = false;
    return false;
    }
 
    // give a chance for the parent parameters to get processed
    return RRTConnectParameters::endElement(name);
  }
} /* namespace openrave_ompl_bridge */
