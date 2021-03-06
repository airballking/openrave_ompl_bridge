#ifndef OPENRAVE_OMPL_BRIDGE_ROBOT_H
#define OPENRAVE_OMPL_BRIDGE_ROBOT_H

#include <openrave-core.h>
#include <kdl/jacobian.hpp>
#include <vector>

namespace openrave_ompl_bridge
{
  class Robot
  {
    public:
      Robot(OpenRAVE::RobotBasePtr robot);
      ~Robot();
      
      unsigned int getDOF() const;
      const std::vector<double>& getJointValues();
      void setJointValues(const std::vector<double>& joint_values);

      const std::vector<double>& getLowerJointLimits();
      const std::vector<double>& getUpperJointLimits();

      const KDL::Jacobian& getJacobian(const std::vector<double>& joint_values, const std::string& manipulator_name);
      const KDL::Frame& getEndEffectorFrame(const std::string& manipulator_name);

      bool isInSelfCollision();
      
      int getEndEffectorIndex(const std::string& manipulator_name) const;

      OpenRAVE::ConfigurationSpecification getConfigurationSpec();

      OpenRAVE::RobotBaseConstPtr getRobotPointer();

    private:
      OpenRAVE::RobotBasePtr robot_;
      std::vector<double> helper_dof_values_, helper_lower_limits_, helper_upper_limits_;
      KDL::Jacobian jacobian_;
      
      // some temporary variables
      std::vector<double> position_jacobian_, angular_jacobian_;
      KDL::Frame ee_frame;

      // some internal helpers
      OpenRAVE::RobotBase::ManipulatorPtr getManipulator(const std::string& manipulator_name) const;
      void resizeJacobians();
  };

  typedef boost::shared_ptr<Robot> RobotPtr;
  typedef boost::shared_ptr<const Robot> ConstRobotPtr;

} /* namespace openrave_ompl_bridge */

#endif // OPENRAVE_OMPL_BRIDGE_ROBOT_H
