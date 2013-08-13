#include <openrave_ompl_bridge/FeatureConstraints.h>
#include <openrave_ompl_bridge/Conversions.h>

using namespace Eigen;

namespace openrave_ompl_bridge
{

// transform an interaction matrix
Matrix<double, 6, 6> inverse_twist_proj(KDL::Frame f)
{
  // (transposed) Rotation matrix of f
  Matrix3d Rt = Map<Matrix3d>(f.M.data);

  double x = f.p.x(), y = f.p.y(), z = f.p.z();

  // Skew symmetric matrix of p, [p]_x for expressing a cross product
  Matrix3d px;
  px << 0, -z,  y,
        z,  0, -x,
       -y,  x,  0;

  // the inverse twist projection matrix
  Matrix<double, 6, 6> Mi;
  Mi.block<3,3>(0,0) = Rt;
  Mi.block<3,3>(3,3) = Rt;
  Mi.block<3,3>(0,3) = -Rt*px;
  Mi.block<3,3>(3,0) = Matrix3d::Zero();

  return Mi;
}

  FeatureConstraintsTask::FeatureConstraintsTask()
  {
    Constraint::init();  
  }

  void FeatureConstraintsTask::resize(unsigned int num_constraints, unsigned int num_joints)
  {
    kdl_task_values_.resize(num_constraints);
    task_values_.resize(num_constraints);
    constraint_commands_.resize(num_constraints);
    constraint_configurations_.resize(num_constraints);
    Ht_.resize(num_constraints);
    tmp_.resize(num_constraints);
    ydot_.resize(num_constraints);
    weights_.resize(num_constraints);
    chi_des_.resize(num_constraints);
    gains_.resize(num_constraints);

    for(unsigned int i=0; i<num_constraints; i++)
      gains_(i) = 1.0;

    kdl_joint_values_.resize(num_joints);
    joint_values_.resize(num_joints);
    JR_.resize(num_joints);
    qdot_.resize(num_joints);
    solver_.reinitialise(num_constraints, num_joints);
    Wq_ = Eigen::MatrixXd::Identity(num_joints, num_joints);
    A_ = Eigen::MatrixXd::Zero(num_constraints, num_joints);
    Wy_ = Eigen::MatrixXd::Zero(num_constraints, num_constraints);
    H_feature_ = Eigen::MatrixXd::Zero(num_constraints, 6);
  }

  void FeatureConstraintsTask::calculateConstraintValues()
  {
    updatePoseObjectInTool(); 
    KDL::Frame f = pose_object_in_tool_;
    std::cout << "\npose_object_in_tool_:\n";
    std::cout << "p: " << f.p.x() << " " << f.p.y() << " " << f.p.z() << "\n"; 
    double x,y,z,w;
    f.M.GetQuaternion(x,y,z,w);
    std::cout << "M: " << x << " " << y << " " << " " << z << " " << w << "\n\n";
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

  void printJntArray(const std::string& name, const KDL::JntArray& a)
  {
    std::cout << name<< "(SIZE=)" << a.rows() << "\n";
    for(unsigned int i=0; i<a.rows(); i++)
      std::cout << a(i) << " ";
    std::cout << "\n";
  }

  void printVector(const std::string& name, const std::vector<double>& v)
  {
    std::cout << name<< "(SIZE=)" << v.size() << "\n";
    for(unsigned int i=0; i<v.size(); i++)
      std::cout << v[i] << " ";
    std::cout << "\n";
  }

  void limitJntArrayLength(KDL::JntArray& array, double max_length)
  {
    double length=0.0;
    for(unsigned int i=0; i<array.rows(); i++)
      length+= array(i)*array(i);
    length = std::sqrt(length);

    if(length > 0.0)
    for(unsigned int i=0; i<array.rows(); i++)
      array(i) = array(i)*max_length/length;
  }

  void limitJntArrayEntries(KDL::JntArray& array, double max_entry)
  {
    assert(max_entry>0.0);
    for(unsigned int i=0; i<array.rows(); i++)
      if(std::abs(array(i))>max_entry)
        array(i) = array(i)/std::abs(array(i))*max_entry;
  }
  void FeatureConstraintsTask::constrainCurrentConfiguration(unsigned int max_iterations, double derivative_delta, double max_delta_q, const std::string& manipulator_name)
  {
    std::string tool_name = extractToolName();

    for(unsigned int i=0; i<max_iterations; i++)
    {
      if(areConstraintsFulfilled())
      {
        std::cout << "constraints fulfilled after " << i << " iterations\n";
        return;
      }

      updatePoseObjectInTool();

      differentiateConstraints(Ht_, kdl_task_values_, pose_object_in_tool_, constraint_configurations_, derivative_delta, tmp_); 
      control(ydot_, weights_, chi_des_, kdl_task_values_, constraint_commands_, gains_);

      
      assert(robot_);
      // robot jacobian from OpenRAVE: (Base: world_frame, RefPoint: EE_frame)
      JR_ = robot_->getJacobian(getJointValues(), manipulator_name);
      // change RefPoint of jacobian: (Base: world_frame, RefPoint: world_frame)
      JR_.changeRefPoint(-robot_->getEndEffectorFrame(manipulator_name).p);

      H_feature_ = Ht_.data.transpose();
      // change RefPoint and RefBase of interaction matrix (Base: world_frame, RefPoint: world_frame)
      H_feature_ = H_feature_*inverse_twist_proj(environment_->getObjectFrameInWorld(tool_name));
      
      // assemble problem for solver out of interaction matrix and robot jacobian
      A_ = H_feature_ * JR_.data;
      
      // get desired weights into matrix-from
      Wy_.diagonal() = weights_.data;

      // call the solver to get desired joint angles that achieve task and
      // weighted pseudoinverse of A_ to perform Nullspace projection
      solver_.solve(A_, ydot_.data, Wq_, Wy_, qdot_.data);

      // change the robot state as calculated by solver
      assert(kdl_joint_values_.rows() == qdot_.rows());
      limitJntArrayEntries(qdot_, max_delta_q); 
      for(unsigned int i=0; i<qdot_.rows(); i++)
        kdl_joint_values_(i) = kdl_joint_values_(i) - qdot_(i);
    }
  }
 
  void FeatureConstraintsTask::updatePoseObjectInTool()
  {
    if(environment_)
    {
      std::string tool_name = extractToolName();
      std::string object_name = extractObjectName();

      pose_object_in_tool_ = environment_->getTransform(tool_name, object_name);
    }
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

  const std::string& FeatureConstraintsTask::extractToolName()
  {
    assert(constraint_configurations_.size() > 0);
    for(unsigned int i=1; i<constraint_configurations_.size(); i++)
      assert(constraint_configurations_[0].tool_feature.name.compare(constraint_configurations_[i].tool_feature.name)==0);
    return constraint_configurations_[0].tool_feature.name;
  }

  const std::string& FeatureConstraintsTask::extractObjectName()
  {
    assert(constraint_configurations_.size() > 0);
    for(unsigned int i=1; i<constraint_configurations_.size(); i++)
      assert(constraint_configurations_[0].object_feature.name.compare(constraint_configurations_[i].object_feature.name)==0);
    return constraint_configurations_[0].object_feature.name;
  }

} // namespace openrave_ompl_bridge
