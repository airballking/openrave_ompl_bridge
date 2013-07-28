#ifndef OPENRAVE_OMPL_BRIDGE_ROBOT_H
#define OPENRAVE_OMPL_BRIDGE_ROBOT_H

#include <openrave-core.h>

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

      bool isInSelfCollision();

    private:
      OpenRAVE::RobotBasePtr robot_;
      std::vector<double> helper_dof_values_, helper_lower_limits_, helper_upper_limits_;
  };

  typedef boost::shared_ptr<Robot> RobotPtr;
  typedef boost::shared_ptr<const Robot> ConstRobotPtr;

} /* namespace openrave_ompl_bridge */

#endif // OPENRAVE_OMPL_BRIDGE_ROBOT_H
