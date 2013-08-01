#ifndef _OPENRAVE_OMPL_BRIDGE_CBIRRT_SAMPLING_
#define _OPENRAVE_OMPL_BRIDGE_CBIRRT_SAMPLING_

#include <ompl/base/StateSpace.h>

namespace openrave_ompl_bridge
{
  class CBiRRTStateSampler : public ompl::base::StateSampler
  {
    public:
      CBiRRTStateSampler(const ompl::base::StateSpace *space);
     
      virtual void sampleUniform(ompl::base::State *state);
      virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance);
      virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev);

    protected:
      ompl::base::StateSamplerPtr jointspace_sampler_;
  };

  typedef boost::shared_ptr<CBiRRTStateSampler> abc;

} // namespace openrave_ompl_bridge

#endif // _OPENRAVE_OMPL_BRIDGE_CBIRRT_SAMPLING_
