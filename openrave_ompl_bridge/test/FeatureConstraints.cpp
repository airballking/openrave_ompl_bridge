#include <gtest/gtest.h>
#include <openrave_ompl_bridge/FeatureConstraints.h>

using namespace openrave_ompl_bridge;

class FeatureConstraintsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      unsigned int num_constraints = 3;
      true_constraints.commands_.resize(num_constraints);
      false_constraints.commands_.resize(num_constraints);
      for(unsigned int i=0; i<num_constraints; i++)
      {
        true_constraints.commands_.pos_lo(i) = -1.0;
        true_constraints.commands_.pos_hi(i) = 2.5;
        true_constraints.task_values_.push_back(0.25);

        false_constraints.commands_.pos_lo(i) = -1.0;
        false_constraints.commands_.pos_hi(i) = 2.5;
      }
      false_constraints.task_values_.push_back(0.0);
      false_constraints.task_values_.push_back(-2.0);
      false_constraints.task_values_.push_back(5.5);
    }

    virtual void TearDown()
    {
    } 

    FeatureConstraintsTask true_constraints, false_constraints;
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
