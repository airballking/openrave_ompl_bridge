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

    // updating the constraint values after joint-space interpolation
    updateConstraintValues(state);
  }

  bool CBiRRTSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
  {
    const ompl::base::CompoundState *cstate1 = static_cast<const ompl::base::CompoundState*>(state1);
    const ompl::base::CompoundState *cstate2 = static_cast<const ompl::base::CompoundState*>(state2);
    return components_[0]->equalStates(cstate1->components[0], cstate2->components[0]);
  }

  void CBiRRTSpace::setTaskFunction(const TaskFunctionFn &task_function)
  {
    task_function_ = task_function;
  }

  void CBiRRTSpace::updateConstraintValues(ompl::base::State *state) const
  {
    assert(task_function_);
    
    task_function_(state);
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
