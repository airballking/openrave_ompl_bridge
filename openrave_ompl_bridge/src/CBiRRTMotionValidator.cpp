#include <openrave_ompl_bridge/CBiRRTMotionValidator.h>
#include <queue>

namespace openrave_ompl_bridge
{
  bool CBiRRTMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const
  {
    /* assume motion starts in a valid configuration so s1 is valid */
   
    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);
   
    if (nd > 1)
    {
      /* temporary storage for the checked state */
      ompl::base::State *test = si_->allocState();
     
      for (int j = 1 ; j < nd ; ++j)
      {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        if (!si_->isValid(test) || !stateSpace_->as<openrave_ompl_bridge::CBiRRTSpace>()->fulfillsPathConstraints(test))
        {
          lastValid.second = (double)(j - 1) / (double)nd;
          if (lastValid.first)
          stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
          result = false;
          break;
        }
      }
      si_->freeState(test);
    }
   
    if (result)
      if (!si_->isValid(s2) || !stateSpace_->as<openrave_ompl_bridge::CBiRRTSpace>()->fulfillsPathConstraints(s2))
      {
        lastValid.second = (double)(nd - 1) / (double)nd;
        if (lastValid.first)
          stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
        result = false;
      }
   
    if (result)
      valid_++;
    else
      invalid_++;
   
    return result;
  }
   
  bool CBiRRTMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
  {
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2) || !stateSpace_->as<openrave_ompl_bridge::CBiRRTSpace>()->fulfillsPathConstraints(s2))
    {
      invalid_++;
      return false;
    }
   
    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);
   
    /* initialize the queue of test positions */
    std::queue< std::pair<int, int> > pos;
    if (nd >= 2)
    {
      pos.push(std::make_pair(1, nd - 1));
     
      /* temporary storage for the checked state */
      ompl::base::State *test = si_->allocState();
     
      /* repeatedly subdivide the path segment in the middle (and check the middle) */
      while (!pos.empty())
      {
        std::pair<int, int> x = pos.front();
       
        int mid = (x.first + x.second) / 2;
        stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);
       
        if (!si_->isValid(test) || !stateSpace_->as<openrave_ompl_bridge::CBiRRTSpace>()->fulfillsPathConstraints(test))
        {
          result = false;
          break;
        }
       
        pos.pop();
       
        if (x.first < mid)
          pos.push(std::make_pair(x.first, mid - 1));
        if (x.second > mid)
          pos.push(std::make_pair(mid + 1, x.second));
      }
     
      si_->freeState(test);
    }
   
    if (result)
      valid_++;
    else
      invalid_++;
   
    return result;
  }

  void CBiRRTMotionValidator::defaultSettings(void)
  {
    stateSpace_ = si_->getStateSpace().get();
    if (!stateSpace_)
      throw ompl::Exception("No state space for motion validator");
  }

} // namespace openrave_ompl_bridge
