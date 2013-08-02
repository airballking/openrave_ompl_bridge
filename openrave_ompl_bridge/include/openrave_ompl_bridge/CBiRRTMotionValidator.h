#ifndef _OPENRAVE_OMPL_BRIDGE_CBIRRT_MOTION_VALIDATOR_
#define _OPENRAVE_OMPL_BRIDGE_CBIRRT_MOTION_VALIDATOR_
 
#include <openrave_ompl_bridge/CBiRRTSpace.h>
#include <ompl/base/DiscreteMotionValidator.h>

namespace openrave_ompl_bridge
{
  class CBiRRTMotionValidator : public ompl::base::DiscreteMotionValidator
  {
    public:
    CBiRRTMotionValidator(ompl::base::SpaceInformation* si) : ompl::base::DiscreteMotionValidator(si)
    {
    }
   
    CBiRRTMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::DiscreteMotionValidator(si)
    {
    }
   
    virtual ~CBiRRTMotionValidator(void)
    {
    }
   
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const;

    private:
      ompl::base::StateSpace *stateSpace_;
      void defaultSettings(void);
  };

} // namespace openrave_ompl_bridge

#endif // _OPENRAVE_OMPL_BRIDGE_CBIRRT_MOTION_VALIDATOR_
