#include <iostream>
#include <openrave_ompl_bridge/Robot.h>
#include <openrave_ompl_bridge/Conversions.h>

using namespace OpenRAVE;

namespace openrave_ompl_bridge
{
  Robot::Robot(RobotBasePtr robot)
  {
    assert(robot);

    robot_ = robot;
    jacobian_.resize(getDOF());
    position_jacobian_.resize(getDOF()*3);
    angular_jacobian_.resize(getDOF()*3);
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
    assert(robot_);

    return robot_->CheckSelfCollision();
  }

  OpenRAVE::ConfigurationSpecification Robot::getConfigurationSpec()
  {
    assert(robot_);

    return robot_->GetActiveConfigurationSpecification();
  }

  OpenRAVE::RobotBaseConstPtr Robot::getRobotPointer()
  {
    return robot_;
  }

  // Returns the robot Jacobian in configuration 'joint_values', i.e. the Jacobian of the end-effector frame of 
  // 'manipulator_name' w.r.t. to the OpenRAVE world-frame.
  const KDL::Jacobian& Robot::getJacobian(const std::vector<double>& joint_values, const std::string& manipulator_name)
  {
    assert(robot_);
    assert(getDOF() == joint_values.size());
    
    setJointValues(joint_values);

    if(jacobian_.columns() != getDOF())
      jacobian_.resize(getDOF());

    int ee_index = getEndEffectorIndex(manipulator_name);


    position_jacobian_.clear();
    angular_jacobian_.clear();

    robot_->CalculateActiveJacobian(ee_index, getManipulator(manipulator_name)->GetEndEffectorTransform().trans, position_jacobian_);
    robot_->CalculateActiveAngularVelocityJacobian(ee_index, angular_jacobian_);
  
    toKDL(jacobian_, position_jacobian_, angular_jacobian_);

    return jacobian_;
  }

  const KDL::Frame& Robot::getEndEffectorFrame(const std::string& manipulator_name)
  {
    OpenRAVE::Transform ee_transform = getManipulator(manipulator_name)->GetEndEffector()->GetTransform();
    toKDL(ee_frame, ee_transform);
    return ee_frame; 
  }

  int Robot::getEndEffectorIndex(const std::string& manipulator_name) const
  {
    return getManipulator(manipulator_name)->GetEndEffector()->GetIndex();
  }

  void Robot::resizeJacobians()
  {
    if(position_jacobian_.size() != getDOF()*3)
      position_jacobian_.resize(getDOF()*3);

    if(angular_jacobian_.size() != getDOF()*3)
      angular_jacobian_.resize(getDOF()*3);
  }

  OpenRAVE::RobotBase::ManipulatorPtr Robot::getManipulator(const std::string& manipulator_name) const
  {
    assert(robot_);

    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips = robot_->GetManipulators();
    unsigned int i;
    for(i=0; i<manips.size(); i++)
    {
      if(manips[i]->GetName().compare(manipulator_name)==0)
        break;
    }

    // HACK: hit the assertion in case the manipulator does not exist
    assert(i<manips.size());
    return manips[i];
  }

} /* namespace openrave_ompl_bridge */
