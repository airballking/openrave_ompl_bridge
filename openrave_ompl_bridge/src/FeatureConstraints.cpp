#include <openrave_ompl_bridge/FeatureConstraints.h>
#include <feature_constraints/Conversions.h>

namespace openrave_ompl_bridge
{
  FeatureConstraintsTask::FeatureConstraintsTask()
  {
    Constraint::init();  
  }

  void FeatureConstraintsTask::init(unsigned int num_constraints)
  {
    kdl_task_values_.resize(num_constraints);
    kdl_joint_values_.resize(num_constraints);
    task_values_.resize(num_constraints);
    joint_values_.resize(num_constraints);
    commands_.resize(num_constraints);
  }

  const std::vector<double>& FeatureConstraintsTask::calculateConstraintValues()
  {
    evaluateConstraints(kdl_task_values_, pose_object_in_tool_, constraint_configurations_);
    toMsg(kdl_task_values_, task_values_);
    return task_values_;
  }

  bool FeatureConstraintsTask::areConstraintsFulfilled() const
  {
    assert(kdl_task_values_.rows() == commands_.pos_lo.rows());
    assert(kdl_task_values_.rows() == commands_.pos_hi.rows());

    for(unsigned int i=0; i<kdl_task_values_.rows(); i++)
    {
      // return false as soon as one of the constraints is not fulfilled
      if(kdl_task_values_(i) < commands_.pos_lo(i) || kdl_task_values_(i) > commands_.pos_hi(i))
        return false;
    }

    // all of the constraints seemed OK
    return true;
  }

  double FeatureConstraintsTask::distanceFromConstraints() const
  {
    assert(kdl_task_values_.rows() == commands_.pos_lo.rows());
    assert(kdl_task_values_.rows() == commands_.pos_hi.rows());
   
    double sum = 0.0;

    for(unsigned int i=0; i<kdl_task_values_.rows(); i++)
    {
      // every not-fulfilled constraint adds to the squard sum
      if(kdl_task_values_(i) < commands_.pos_lo(i) || kdl_task_values_(i) > commands_.pos_hi(i))
      {
        double d1 = commands_.pos_lo(i) - kdl_task_values_(i);
        double d2 = commands_.pos_hi(i) - kdl_task_values_(i);
        sum += std::min(d1*d1, d2*d2);
      }
    }
 
    return std::sqrt(sum);
  }

  const std::vector<double>& FeatureConstraintsTask::constrainConfiguration(const std::vector<double>& start_joint_values)
  {
    return joint_values_;
  }
 
  void FeatureConstraintsTask::setJointValues(const std::vector<double>& joint_values)
  {
    assert(kdl_joint_values_.rows() == joint_values.size());
    
    fromMsg(joint_values, kdl_joint_values_);
  }

  const std::vector<double>& FeatureConstraintsTask::getJointValues() const
  {
    assert(kdl_joint_values_.rows() == joint_values_.size());
   
    return joint_values_;
  }

  void FeatureConstraintsTask::setTaskValues(const std::vector<double>& task_values)
  {
    assert(kdl_task_values_.rows() == task_values.size());
    
    fromMsg(task_values, kdl_task_values_);
  }

  const std::vector<double>& FeatureConstraintsTask::getTaskValues() const
  {
    assert(kdl_task_values_.rows() == task_values_.size());
   
    return task_values_;
  } 


} // namespace openrave_ompl_bridge
