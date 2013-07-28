#include <openrave_ompl_bridge/Robot.h>

using namespace OpenRAVE;

namespace openrave_ompl_bridge
{
  Robot::Robot(RobotBasePtr robot)
  {
    robot_ = robot;
  }

  Robot::~Robot()
  {
  }

  unsigned int Robot::getDOF() const
  {
    assert(robot_);

    return robot_->GetActiveDOF();
  }

  const std::vector<double>& Robot::getJointValues()
  {
    assert(robot_);
   
    robot_->GetActiveDOFValues(helper_dof_values_);
    return helper_dof_values_;
  }
      
  void Robot::setJointValues(const std::vector<double>& joint_values)
  {
    assert(robot_);
    assert(joint_values.size() == getDOF());

    robot_->SetActiveDOFValues(joint_values);
  }

  const std::vector<double>& Robot::getLowerJointLimits()
  {
    assert(robot_);

    robot_->GetActiveDOFLimits(helper_lower_limits_, helper_upper_limits_);
    return helper_lower_limits_;
  }
      
  const std::vector<double>& Robot::getUpperJointLimits()
  {
    assert(robot_);

    robot_->GetActiveDOFLimits(helper_lower_limits_, helper_upper_limits_);
    return helper_upper_limits_;
  }

  bool Robot::isInSelfCollision()
  {
    return robot_->CheckSelfCollision();
  }
} /* namespace openrave_ompl_bridge */
