#include <vector>
#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Controller.h>
#include <kdl/frames.hpp>
#include <openrave_ompl_bridge/Robot.h>
#include <openrave_ompl_bridge/Environment.h>

namespace openrave_ompl_bridge
{

  // I'm pulling the 'Kent'-trick here, i.e. 'stuff all the shit into a class, put some heavy function
  // on top of it, and forget about it." Lazy times in Pittsburgh, Summer of 2013. ;) Sorry for the bad design.
  class FeatureConstraintsTask
  {
    public:
      FeatureConstraintsTask(); 

      // the functionality we're offering
      void init(unsigned int num_constraints);
      bool areConstraintsFulfilled() const;
      double distanceFromConstraints() const;
      const std::vector<double>& calculateConstraintValues(); 
      const std::vector<double>& constrainConfiguration(const std::vector<double>& start_joint_values);
      
      // getters and setters
      void setJointValues(const std::vector<double>& joint_values);
      const std::vector<double>& getJointValues() const;  

      void setTaskValues(const std::vector<double>& task_values);
      const std::vector<double>& getTaskValues() const; 

      void setObjectPose(const KDL::Frame& pose_object_in_tool);
      const KDL::Frame& getObjectPose();

      void setToolPose(const KDL::Frame& pose_tool_in_EE);
      const KDL::Frame& getToolPose();

      void setRobot(const RobotPtr robot);
      RobotPtr getRobot();

      void setEnvironment(const EnvironmentPtr environment);
      EnvironmentPtr getEnvironment();

      void setConstraints(const std::vector<Constraint>& constraints);
      const std::vector<Constraint>& getConstraints() const;

      void setCommands(const Ranges& commands);
      const Ranges& getCommands() const;

   private:
      // members holding task and joint-values
      // KDL representation is always the current internal representation
      // std::vector representations are temporary variables
      std::vector<double> joint_values_, task_values_;
      KDL::JntArray kdl_joint_values_, kdl_task_values_;

      // homogeneous transforms necessary to perform our calculations
      // need to know where the object is w.r.t to the tool
      KDL::Frame pose_object_in_tool_; 
      // need to know where the tool is w.r.t. to the end-effector
      KDL::Frame pose_tool_in_EE_;

      // proxy-objects to access openrave-stuff
      RobotPtr robot_;
      EnvironmentPtr environment_;

      // the actual constraints
      std::vector<Constraint> constraint_configurations_;
      Ranges constraint_commands_;
  }; 

  class FeatureConstraintsPlanningTask
  {
    public:
      FeatureConstraintsTask path_constraints_;
      FeatureConstraintsTask goal_constraints_;

      // some convenience functions for stuff that's equal in both constraint sets
      void setRobot(const RobotPtr& robot);
      void setEnvironment(const EnvironmentPtr& environment);
      void setToolPose(const KDL::Frame& pose_tool_in_EE);
      void setObjectPose(const KDL::Frame& pose_object_in_tool);
  };
} // namespace openrave_ompl_bridge
