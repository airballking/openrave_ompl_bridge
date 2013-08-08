#include <openrave_ompl_bridge/FeatureConstraints.h>

namespace openrave_ompl_bridge
{
  const std::vector<double>& FeatureConstraintsTask::evaluateConstraints(const std::vector<double>& joint_values)
  {
    return task_values_;
  }

  bool FeatureConstraintsTask::areConstraintsFulfilled() const
  {
    assert(task_values_.size() == commands_.pos_lo.rows());
    assert(task_values_.size() == commands_.pos_hi.rows());

    for(unsigned int i=0; i<task_values_.size(); i++)
    {
      // return false as soon as one of the constraints is not fulfilled
      if(task_values_[i] < commands_.pos_lo(i) || task_values_[i] > commands_.pos_hi(i))
        return false;
    }

    // all of the constraints seemed OK
    return true;
  }

  double FeatureConstraintsTask::distanceFromConstraints() const
  {
    assert(task_values_.size() == commands_.pos_lo.rows());
    assert(task_values_.size() == commands_.pos_hi.rows());
   
    double sum = 0.0;

    for(unsigned int i=0; i<task_values_.size(); i++)
    {
      // every not-fulfilled constraint adds to the squard sum
      if(task_values_[i] < commands_.pos_lo(i) || task_values_[i] > commands_.pos_hi(i))
      {
        double d1 = commands_.pos_lo(i) - task_values_[i];
        double d2 = commands_.pos_hi(i) - task_values_[i];
        sum += std::min(d1*d1, d2*d2);
      }
    }
 
    return std::sqrt(sum);
  }

  const std::vector<double>& FeatureConstraintsTask::constrainConfiguration(const std::vector<double>& start_joint_values)
  {
    return joint_values_;
  }
 
} // namespace openrave_ompl_bridge
