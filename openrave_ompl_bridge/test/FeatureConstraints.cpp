#include <gtest/gtest.h>
#include <openrave_ompl_bridge/FeatureConstraints.h>

using namespace openrave_ompl_bridge;

class FeatureConstraintsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      unsigned int num_constraints = 3;
      true_constraints.init(num_constraints);
      false_constraints.init(num_constraints);
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

      Feature oven, bottle;
      oven.name = "oven";
      oven.pos = KDL::Vector(0.0, 0.0, 0.0);
      oven.dir = KDL::Vector(0.0, 0.0, 1.0);
      oven.contact_dir = KDL::Vector(1.0, 0.0, 0.0);
      bottle.name = "bottle";
      bottle.pos = KDL::Vector(0.0, 0.0, 0.0);
      bottle.dir = KDL::Vector(0.0, 0.0, 1.0);
      bottle.contact_dir = KDL::Vector(1.0, 0.0, 0.0);
 
      Constraint c1, c2, c3;
      c1.name = "height of bottle over oven";
      c1.setFunction("height");
      c1.tool_feature = bottle;
      c1.object_feature = oven;
    
      c2.name = "distance of bottle over oven";
      c2.setFunction("distance");
      c2.tool_feature = bottle;
      c2.object_feature = oven;

      c3.name = "bottle upright";
      c3.setFunction("perpendicular");
      c3.tool_feature = bottle;
      c3.object_feature = oven;

      constraints.init(3);
      std::vector<Constraint> constraint_vector;
      constraint_vector.push_back(c1);
      constraint_vector.push_back(c2);
      constraint_vector.push_back(c3);
      constraints.setConstraints(constraint_vector);
      constraints.setObjectPose(KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(1.0, 1.0, 0.75)));
    }

    virtual void TearDown()
    {
    } 

    FeatureConstraintsTask true_constraints, false_constraints, constraints;
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
  std::vector<double> values = constraints.calculateConstraintValues();
  ASSERT_EQ(values.size(), 3);
  EXPECT_DOUBLE_EQ(0.75, values[0]);
  EXPECT_DOUBLE_EQ(std::sqrt(2.0), values[1]);
  EXPECT_DOUBLE_EQ(1.0, values[2]);
}
