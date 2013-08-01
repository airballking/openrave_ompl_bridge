#include <openrave_ompl_bridge/CBiRRTSpace.h>

namespace openrave_ompl_bridge
{
  void CBiRRTSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
  {
    const ompl::base::CompoundState *cfrom = static_cast<const ompl::base::CompoundState*>(from);
    const ompl::base::CompoundState *cto = static_cast<const ompl::base::CompoundState*>(to);
    ompl::base::CompoundState *cstate = static_cast<ompl::base::CompoundState*>(state);
    // using regular joint-space interpolation for joint-space part of state
    components_[0]->interpolate(cfrom->components[0], cto->components[0], t, cstate->components[0]);

    // setting some weird stuff for the constraints, should not interfere with distance and equals
    ompl::base::RealVectorStateSpace::StateType* realVectorState = cstate->components[1]->as<ompl::base::RealVectorStateSpace::StateType>();
    for(unsigned int i=0; i<getNumberOfConstraints(); i++)
      realVectorState->values[i] = 1.0;

  }

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
