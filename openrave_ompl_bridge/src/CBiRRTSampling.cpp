#include <openrave_ompl_bridge/CBiRRTSampling.h>
#include <openrave_ompl_bridge/CBiRRTSpace.h>

namespace openrave_ompl_bridge
{
  CBiRRTStateSampler::CBiRRTStateSampler(const ompl::base::StateSpace *space) : ompl::base::StateSampler(space) 
  {
    assert(space);

    ompl::base::StateSpacePtr jointspace = space->as<openrave_ompl_bridge::CBiRRTSpace>()->getJointSpace();
    jointspace_sampler_ = jointspace->allocStateSampler();
  }

  void CBiRRTStateSampler::sampleUniform(ompl::base::State *state)
  {
    assert(state);

    ompl::base::State **comps = state->as<ompl::base::CompoundState>()->components;
    jointspace_sampler_->sampleUniform(comps[0]);
  }

  void CBiRRTStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
  {
    assert(state);
    assert(near);

    ompl::base::State **comps = state->as<ompl::base::CompoundState>()->components;
    ompl::base::State **nearComps = near->as<ompl::base::CompoundState>()->components;
    jointspace_sampler_->sampleUniformNear(comps[0], nearComps[0], distance);
  } 
   
  void CBiRRTStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
  {
    assert(state);
    assert(mean);

    ompl::base::State **comps = state->as<ompl::base::CompoundState>()->components;
    ompl::base::State **meanComps = mean->as<ompl::base::CompoundState>()->components;
    jointspace_sampler_->sampleGaussian(comps[0], meanComps[0], stdDev);
  }
} // namespace openrave_ompl_bridge

