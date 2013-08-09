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


      // input parameters for the various functions
      std::vector<Constraint> constraint_configurations_;
      Ranges commands_;
      KDL::Frame pose_object_in_tool_; 
      KDL::Frame pose_tool_in_EE_;
      RobotPtr robot_;
      EnvironmentPtr environment_;

    private:
      // members holding task and joint-values
      // KDL representation is always the current internal representation
      // std::vector representations are temporary variables
      std::vector<double> joint_values_, task_values_;
      KDL::JntArray kdl_joint_values_, kdl_task_values_;
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
