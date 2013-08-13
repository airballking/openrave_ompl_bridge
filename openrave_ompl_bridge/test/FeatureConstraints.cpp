#include <gtest/gtest.h>
#include <openrave_ompl_bridge/FeatureConstraints.h>

using namespace openrave_ompl_bridge;

class FeatureConstraintsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      unsigned int num_constraints = 3;
      unsigned int num_joints = 7;
      true_constraints.resize(num_constraints, num_joints);
      false_constraints.resize(num_constraints, num_joints);
      std::vector<double> task_values;
      Ranges true_cmd, false_cmd;
      true_cmd.resize(num_constraints);
      false_cmd.resize(num_constraints);
      for(unsigned int i=0; i<num_constraints; i++)
      {
        true_cmd.pos_lo(i) = -1.0;
        true_cmd.pos_hi(i) = 2.5;
        task_values.push_back(0.25);

        false_cmd.pos_lo(i) = -1.0;
        false_cmd.pos_hi(i) = 2.5;
      }
      true_constraints.setTaskValues(task_values);
      task_values[0] = (0.0);
      task_values[1] = (-2.0);
      task_values[2] = (5.5);
      false_constraints.setTaskValues(task_values);

      true_constraints.setCommands(true_cmd);
      false_constraints.setCommands(false_cmd);

      Feature pan, bottle;
      pan.name = "pan";
      pan.pos = KDL::Vector(0.0, 0.0, 0.0);
      pan.dir = KDL::Vector(0.0, 0.0, 1.0);
      pan.contact_dir = KDL::Vector(1.0, 0.0, 0.0);
      bottle.name = "bottle";
      bottle.pos = KDL::Vector(0.0, 0.0, 0.0);
      bottle.dir = KDL::Vector(0.0, 0.0, 1.0);
      bottle.contact_dir = KDL::Vector(1.0, 0.0, 0.0);
 
      Constraint c1, c2, c3;
      c1.name = "height of bottle over pan";
      c1.setFunction("height");
      c1.tool_feature = bottle;
      c1.object_feature = pan;
    
      c2.name = "distance of bottle over pan";
      c2.setFunction("distance");
      c2.tool_feature = bottle;
      c2.object_feature = pan;

      c3.name = "bottle upright";
      c3.setFunction("perpendicular");
      c3.tool_feature = bottle;
      c3.object_feature = pan;

      constraints.resize(3, num_joints);
      std::vector<Constraint> constraint_vector;
      constraint_vector.push_back(c1);
      constraint_vector.push_back(c2);
      constraint_vector.push_back(c3);
      constraints.setConstraints(constraint_vector);
      constraints.setObjectPose(KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(1.0, 1.0, 0.75)));

      Ranges cmd;
      cmd.resize(3);
      // height
      cmd.weight(0) = 1.0;
      cmd.pos_lo(0) = 0.1;
      cmd.pos_hi(0) = 2.0; 
      // distance
      cmd.weight(1) = 1.0;
      cmd.pos_lo(1) = 0.0;
      cmd.pos_hi(1) = 0.1; 
      // upright
      cmd.weight(2) = 1.0;
      cmd.pos_lo(2) = 0.95;
      cmd.pos_hi(2) = 1.2; 
      constraints.setCommands(cmd);
    }

    virtual void TearDown()
    {
    } 

    FeatureConstraintsTask true_constraints, false_constraints, constraints;
    EnvironmentPtr env;
    RobotPtr robot;
};

TEST_F(FeatureConstraintsTest, areConstraintsFulfilled)
{
  EXPECT_TRUE(true_constraints.areConstraintsFulfilled());
  EXPECT_FALSE(false_constraints.areConstraintsFulfilled());
}

TEST_F(FeatureConstraintsTest, distanceFromConstraints)
{
  EXPECT_DOUBLE_EQ(true_constraints.distanceFromConstraints(), 0.0);
  EXPECT_DOUBLE_EQ(false_constraints.distanceFromConstraints(), std::sqrt(10.0));
}

TEST_F(FeatureConstraintsTest, calculateConstraintValues)
{
  constraints.calculateConstraintValues();
  std::vector<double> values = constraints.getTaskValues();
  ASSERT_EQ(values.size(), 3);
  EXPECT_DOUBLE_EQ(0.75, values[0]);
  EXPECT_DOUBLE_EQ(std::sqrt(2.0), values[1]);
  EXPECT_DOUBLE_EQ(1.0, values[2]);
}

TEST_F(FeatureConstraintsTest, getFeatureNames)
{
  EXPECT_STREQ(constraints.extractToolName().c_str(), "bottle");
  EXPECT_STREQ(constraints.extractObjectName().c_str(), "pan");
}

TEST_F(FeatureConstraintsTest, constrainConfiguration)
{
  // setting up environment and robot
  OpenRAVE::RaveInitialize();
  OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
  OpenRAVE::RobotBasePtr herb = env->ReadRobotXMLFile("robots/herb2_padded_nosensors.robot.xml");
  env->Add(herb);
 
  OpenRAVE::RobotBase::ManipulatorPtr right_arm;
  std::vector<OpenRAVE::RobotBase::ManipulatorPtr> herb_manipulators = herb->GetManipulators();
  for (unsigned int i=0; i<herb_manipulators.size(); i++)
  {
    if(herb_manipulators[i] && (herb_manipulators[i]->GetName().compare("right_wam") == 0))
    {
      right_arm = herb_manipulators[i];
    }
  }
 
  herb->SetActiveManipulator(right_arm);
  herb->SetActiveDOFs(right_arm->GetArmIndices());

  // putting the bottle in the robot's right hand
  OpenRAVE::KinBodyPtr bottle_body = OpenRAVE::RaveCreateKinBody(env, "");
  ASSERT_TRUE(env->ReadKinBodyURI(bottle_body, "objects/household/fuze_bottle.kinbody.xml"));
  bottle_body->SetName("bottle");
  env->Add(bottle_body);

  OpenRAVE::geometry::RaveTransformMatrix<OpenRAVE::dReal> m;
  m.rotfrommat(-0.86354026,  0.50427591,  0.00200643, 
               -0.25814712, -0.44547139,  0.85727201,
                0.43319543,  0.73977094,  0.51485985);
  m.trans.x = 1.10322189;
  m.trans.y = -0.24693695;
  m.trans.z = 0.87205762;
  bottle_body->SetTransform(OpenRAVE::Transform(m));

  OpenRAVE::Transform bottle_transform1 = bottle_body->GetTransform();

  ASSERT_TRUE(herb->Grab(bottle_body));

  // adding a flying pan
  OpenRAVE::KinBodyPtr pan_body =  OpenRAVE::RaveCreateKinBody(env, "");
  ASSERT_TRUE(env->ReadKinBodyURI(pan_body, "objects/household/pan.kinbody.xml"));
  pan_body->SetName("pan");
  env->Add(pan_body);

  m.rotfrommat(2.89632833e-03,   9.99995806e-01,   1.19208782e-07,
               -9.99995806e-01,   2.89632833e-03,  -1.18864027e-07,
               -1.19208797e-07,  -1.18864013e-07,   1.00000000e+00);
  m.trans.x = 0.58;
  m.trans.y = -0.02;
  m.trans.z = 0.33;
  pan_body->SetTransform(OpenRAVE::Transform(m));

  // giving constraints access to environment and robot
  RobotPtr robot = RobotPtr(new Robot(herb));
  EnvironmentPtr environment = EnvironmentPtr(new Environment(env));
  constraints.setRobot(robot);
  constraints.setEnvironment(environment);

  // first test
  constraints.calculateConstraintValues();
  EXPECT_FALSE(constraints.areConstraintsFulfilled()); 

  // moving robot and bottle to new start configuration
  std::vector<double> new_config;
  new_config.push_back(2.0);
  new_config.push_back(1.0);
  new_config.push_back(-0.8);
  new_config.push_back(1.0);
  new_config.push_back(-1.0);
  new_config.push_back(0.0);
  new_config.push_back(0.0);

  robot->setJointValues(new_config);
  constraints.setJointValues(robot->getJointValues());
  constraints.calculateConstraintValues();

  // test we've really moved
  OpenRAVE::Transform bottle_transform2 = bottle_body->GetTransform();
  ASSERT_GT((bottle_transform1.trans - bottle_transform2.trans).lengthsqr3(), 0.1);
  ASSERT_GT((bottle_transform1.rot - bottle_transform2.rot).lengthsqr4(), 0.1);

  // actual projection test
  ASSERT_EQ(robot->getDOF(), constraints.getJointValues().size());

  std::cout << "\nprior to constraining:\n";
  std::vector<double> tmp = constraints.getTaskValues();
  for(unsigned int i=0; i<tmp.size(); i++)
  {
    std::cout << tmp[i] << " ";
  } 

  constraints.constrainCurrentConfiguration();
  std::cout << "\npost to constraining:\n";
  tmp = constraints.getTaskValues();
  for(unsigned int i=0; i<tmp.size(); i++)
  {
    std::cout << tmp[i] << " ";
  } 
  std::cout << "\n";

  EXPECT_TRUE(constraints.areConstraintsFulfilled());

  OpenRAVE::RaveDestroy();
}
