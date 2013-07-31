#include <openrave_ompl_bridge/CBiRRTSpace.h>

namespace openrave_ompl_bridge
{
  ompl::base::State* CBiRRTSpace::allocState(void) const
  {
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
  }

  void CBiRRTSpace::freeState(ompl::base::State *state) const
  {
    CompoundStateSpace::freeState(state);
  }
} // namespace openrave_ompl_bridge
