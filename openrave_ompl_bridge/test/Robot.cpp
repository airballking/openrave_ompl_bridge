#include <gtest/gtest.h>
#include <math.h>
#include <openrave-core.h>
#include <openrave_ompl_bridge/Robot.h>
#include <openrave_ompl_bridge/Conversions.h>

using namespace openrave_ompl_bridge;

class RobotTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      OpenRAVE::RaveInitialize();
      env = OpenRAVE::RaveCreateEnvironment();
      herb = env->ReadRobotXMLFile("robots/herb2_padded_nosensors.robot.xml");
      env->Add(herb);
      std::vector<OpenRAVE::RobotBase::ManipulatorPtr> herb_manipulators = herb->GetManipulators();
      for (unsigned int i=0; i<herb_manipulators.size(); i++)
      {
        if(herb_manipulators[i] && (herb_manipulators[i]->GetName().compare("right_wam") == 0))
        {
          right_arm = herb_manipulators[i];
        }
        else if(herb_manipulators[i] && (herb_manipulators[i]->GetName().compare("left_wam") == 0))
        {
          left_arm = herb_manipulators[i];
        }
      }
      test_robot = RobotPtr(new Robot(herb));
    }

    virtual void TearDown()
    {
      OpenRAVE::RaveDestroy();
    }

    OpenRAVE::EnvironmentBasePtr env;
    OpenRAVE::RobotBasePtr herb; 
    OpenRAVE::RobotBase::ManipulatorPtr right_arm, left_arm;
    RobotPtr test_robot;
};

TEST_F(RobotTest, GetRobot)
{
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(test_robot);

  EXPECT_EQ(24, test_robot->getDOF());

  herb->SetActiveDOFs(right_arm->GetArmIndices());
  EXPECT_EQ(7, test_robot->getDOF());

  std::vector<int> no_indices;
  herb->SetActiveDOFs(no_indices);
  EXPECT_EQ(0, test_robot->getDOF());
}

TEST_F(RobotTest, SettingAndGettingJointValues)
{
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(test_robot);

  herb->SetActiveDOFs(right_arm->GetArmIndices());
  std::vector<double> new_joint_values, read_joint_values;
  for (unsigned int i=0; i<7; i++)
    new_joint_values.push_back(1.0);
  
  ASSERT_EQ(test_robot->getDOF(), new_joint_values.size());
  test_robot->setJointValues(new_joint_values);
  read_joint_values = test_robot->getJointValues();

  ASSERT_EQ(new_joint_values.size(), read_joint_values.size());
  for (unsigned int i=0; i<read_joint_values.size(); i++)
    EXPECT_DOUBLE_EQ(new_joint_values[i], read_joint_values[i]);
}

TEST_F(RobotTest, GettingJointLimits)
{
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(test_robot);

  herb->SetActiveDOFs(right_arm->GetArmIndices());
  std::vector<double> lower_limits = test_robot->getLowerJointLimits();
  std::vector<double> upper_limits = test_robot->getUpperJointLimits();

  EXPECT_EQ(test_robot->getDOF(), lower_limits.size());
  EXPECT_EQ(7, lower_limits.size());
  ASSERT_EQ(lower_limits.size(), upper_limits.size());

  for (unsigned int i=0; i<lower_limits.size(); i++)
    EXPECT_LE(lower_limits[i], upper_limits[i]);
}

TEST_F(RobotTest, SelfCollision)
{
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(left_arm);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(test_robot);

    
  double no_col[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double col[] = {4.8, 0.0, 0.0, 2.0, -2.0, 0.0, 0.0};
  double left[] = {0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> no_collision_config (no_col, no_col + sizeof(no_col) / sizeof(no_col[0]) );
  std::vector<double> collision_config (col, col + sizeof(col) / sizeof(col[0]) );
  std::vector<double> left_arm_config (left, left + sizeof(left) / sizeof(left[0]) );

  herb->SetActiveDOFs(left_arm->GetArmIndices());
  test_robot->setJointValues(left_arm_config);

  herb->SetActiveDOFs(right_arm->GetArmIndices());
  test_robot->setJointValues(no_collision_config);
  ASSERT_FALSE(test_robot->isInSelfCollision());

  herb->SetActiveDOFs(right_arm->GetArmIndices());
  test_robot->setJointValues(collision_config);
  ASSERT_TRUE(test_robot->isInSelfCollision());
}

TEST_F(RobotTest, Jacobian)
{
  ASSERT_TRUE(right_arm);
  std::vector<double> non_singular_conf;
  non_singular_conf.push_back(0.9);
  non_singular_conf.push_back(0.0);
  non_singular_conf.push_back(0.0);
  non_singular_conf.push_back(0.0);
  non_singular_conf.push_back(0.0);
  non_singular_conf.push_back(0.0);
  non_singular_conf.push_back(0.0);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  test_robot->setJointValues(non_singular_conf);

  KDL::Jacobian jac;
  jac.resize(test_robot->getDOF());
 
  std::vector<double> jacobian, angular_vel_jacobian;
  right_arm->CalculateJacobian(jacobian);
  right_arm->CalculateAngularVelocityJacobian(angular_vel_jacobian);
  toKDL(jac, jacobian, angular_vel_jacobian);

  KDL::Jacobian jac2 = test_robot->getJacobian(test_robot->getJointValues(), right_arm->GetName());

  double epsilon = std::pow(10, -14);
  for(unsigned int i=0; i<jac.rows(); i++)
  {
    for(unsigned int j=0; j<jac.columns(); j++)
    {
      EXPECT_NEAR(jac(i,j), jac2(i,j), epsilon);
    }
  }  
}
