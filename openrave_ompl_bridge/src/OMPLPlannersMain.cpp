#include <openrave/plugin.h>
#include <openrave_ompl_bridge/OMPLPlannerRRTConnect.h>

using namespace OpenRAVE;

// This file collects all OMPL-Planners wrapped in this package and registers them with OpenRAVE.

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Planner)
    {
      // assertion: someone wants to create a planner plugin
      // add patterns for future OMPL planners below
      if (interfacename == "ompl_rrtconnect")
      {
        return InterfaceBasePtr(new openrave_ompl_bridge::OMPLPlannerRRTConnect(penv));
      }
    }
    // we did not fit requested planner plugin
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    // add any new OMP planner to this list
    info.interfacenames[PT_Planner].push_back("ompl_rttconnect");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    // so far, do nothing
    RAVELOG_INFO("destroying plugin\n");
}

