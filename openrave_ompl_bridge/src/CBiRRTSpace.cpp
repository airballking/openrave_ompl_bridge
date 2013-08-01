#include <openrave_ompl_bridge/CBiRRTSpace.h>

namespace openrave_ompl_bridge
{
  bool CBiRRTSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
  {
    const ompl::base::CompoundState *cstate1 = static_cast<const ompl::base::CompoundState*>(state1);
    const ompl::base::CompoundState *cstate2 = static_cast<const ompl::base::CompoundState*>(state2);
    return components_[0]->equalStates(cstate1->components[0], cstate2->components[0]);
  }

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
