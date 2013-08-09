#include <openrave_ompl_bridge/FeatureConstraints.h>
#include <openrave_ompl_bridge/Conversions.h>

namespace openrave_ompl_bridge
{
  FeatureConstraintsTask::FeatureConstraintsTask()
  {
    Constraint::init();  
  }

  void FeatureConstraintsTask::resize(unsigned int num_constraints)
  {
    kdl_task_values_.resize(num_constraints);
    kdl_joint_values_.resize(num_constraints);
    task_values_.resize(num_constraints);
    joint_values_.resize(num_constraints);
    constraint_commands_.resize(num_constraints);
    constraint_configurations_.resize(num_constraints);
  }

  void FeatureConstraintsTask::calculateConstraintValues()
  {
    evaluateConstraints(kdl_task_values_, pose_object_in_tool_, constraint_configurations_);
  }

  bool FeatureConstraintsTask::areConstraintsFulfilled() const
  {
    assert(kdl_task_values_.rows() == constraint_commands_.pos_lo.rows());
    assert(kdl_task_values_.rows() == constraint_commands_.pos_hi.rows());

    for(unsigned int i=0; i<kdl_task_values_.rows(); i++)
    {
      // return false as soon as one of the constraints is not fulfilled
      if(kdl_task_values_(i) < constraint_commands_.pos_lo(i) || kdl_task_values_(i) > constraint_commands_.pos_hi(i))
        return false;
    }

    // all of the constraints seemed OK
    return true;
  }

  double FeatureConstraintsTask::distanceFromConstraints() const
  {
    assert(kdl_task_values_.rows() == constraint_commands_.pos_lo.rows());
    assert(kdl_task_values_.rows() == constraint_commands_.pos_hi.rows());
   
    double sum = 0.0;

    for(unsigned int i=0; i<kdl_task_values_.rows(); i++)
    {
      // every not-fulfilled constraint adds to the squard sum
      if(kdl_task_values_(i) < constraint_commands_.pos_lo(i) || kdl_task_values_(i) > constraint_commands_.pos_hi(i))
      {
        double d1 = constraint_commands_.pos_lo(i) - kdl_task_values_(i);
        double d2 = constraint_commands_.pos_hi(i) - kdl_task_values_(i);
        sum += std::min(d1*d1, d2*d2);
      }
    }
 
    return std::sqrt(sum);
  }

  void FeatureConstraintsTask::constrainCurrentConfiguration()
  {
    //TODO: implement me
  }
 
  void FeatureConstraintsTask::setJointValues(const std::vector<double>& joint_values)
  {
    toKDL(kdl_joint_values_, joint_values);
  }

  const std::vector<double>& FeatureConstraintsTask::getJointValues()
  {
    assert(kdl_joint_values_.rows() == joint_values_.size());
    toVector(joint_values_, kdl_joint_values_);
    return joint_values_;
  }

  void FeatureConstraintsTask::setTaskValues(const std::vector<double>& task_values)
  {
    toKDL(kdl_task_values_, task_values);
  }

  const std::vector<double>& FeatureConstraintsTask::getTaskValues()
  {
    assert(kdl_task_values_.rows() == task_values_.size());
    toVector(task_values_, kdl_task_values_);
    return task_values_;
  } 

  void FeatureConstraintsTask::setObjectPose(const KDL::Frame& pose_object_in_tool)
  {
    pose_object_in_tool_ = pose_object_in_tool;
  }

  const KDL::Frame& FeatureConstraintsTask::getObjectPose()
  {
    return pose_object_in_tool_;
  }

  void FeatureConstraintsTask::setToolPose(const KDL::Frame& pose_tool_in_EE)
  {
    pose_tool_in_EE_ = pose_tool_in_EE;
  }

  const KDL::Frame& FeatureConstraintsTask::getToolPose()
  {
    return pose_tool_in_EE_;
  }

  void FeatureConstraintsTask::setRobot(const RobotPtr robot)
  {
    robot_ = robot;
  }

  RobotPtr FeatureConstraintsTask::getRobot()
  {
    return robot_;
  }

  void FeatureConstraintsTask::setEnvironment(const EnvironmentPtr environment)
  {
    environment_ = environment;
  }

  EnvironmentPtr FeatureConstraintsTask::getEnvironment()
  {
    return environment_;
  }

  void FeatureConstraintsTask::setConstraints(const std::vector<Constraint>& constraints)
  {
    assert(constraint_configurations_.size() == constraints.size());

    for(unsigned int i=0; i<constraints.size(); i++)
      constraint_configurations_[i] = constraints[i];
  }

  const std::vector<Constraint>& FeatureConstraintsTask::getConstraints() const
  {
    return constraint_configurations_;
  }

  void FeatureConstraintsTask::setCommands(const Ranges& commands)
  {
    assert(constraint_commands_.size() == commands.size());

    constraint_commands_ = commands;
  }

  const Ranges& FeatureConstraintsTask::getCommands() const
  {
    return constraint_commands_;
  }

} // namespace openrave_ompl_bridge
