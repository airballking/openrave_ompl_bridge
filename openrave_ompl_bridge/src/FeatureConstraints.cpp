#include <openrave_ompl_bridge/FeatureConstraints.h>

namespace openrave_ompl_bridge
{
  const std::vector<double>& FeatureConstraintsTask::evaluateConstraints(const std::vector<double>& joint_values)
  {
    return task_values_;
  }

  bool FeatureConstraintsTask::areConstraintsFulfilled()
  {
    return false;
  }

  double FeatureConstraintsTask::distanceFromConstraints()
  {
    return 100.0;
  }

  const std::vector<double>& FeatureConstraintsTask::constrainConfiguration(const std::vector<double>& start_joint_values)
  {
    return joint_values_;
  }
 
} // namespace openrave_ompl_bridge
