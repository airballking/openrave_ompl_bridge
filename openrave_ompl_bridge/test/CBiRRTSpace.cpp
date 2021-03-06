#include <gtest/gtest.h>
#include <vector>
#include <openrave_ompl_bridge/CBiRRTSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>

using namespace openrave_ompl_bridge;

class CBiRRTSpaceTest : public ::testing::Test
{
  public:
    bool MyDummyIsValid(const ompl::base::State *state)
    {
      return true;
    }

  protected:
    virtual void SetUp()
    {
      number_joints = 7;
      number_constraints = 4;
      for (unsigned int i=0; i<number_joints; i++)
      {
        lower_joint_limits.push_back(-1.5);
        upper_joint_limits.push_back(2.2);
        some_joints.push_back((upper_joint_limits[i] - lower_joint_limits[i])/2.0);
      }
      for (unsigned int i=0; i<number_constraints; i++)
      {
        lower_constraint_limits.push_back(-2.0);
        upper_constraint_limits.push_back(2.0);
        some_constraints.push_back((upper_constraint_limits[i] - lower_constraint_limits[i])/2.0);
      }
      my_space = CBiRRTSpacePtr(new CBiRRTSpace(number_joints, number_constraints));
      my_space->setConstraintProjectionFunction(boost::bind(&CBiRRTSpaceTest::MyProjectionFunction, this, _1));
    }

    virtual void TearDown()
    {
    }

    void MyProjectionFunction(ompl::base::State *state)
    {
      assert(state);

      CBiRRTSpace::StateType* cbirrt_state = state->as<CBiRRTSpace::StateType>();
      assert(cbirrt_state);

      for(unsigned int i=0; i<my_space->getNumberOfConstraints(); i++)
        cbirrt_state->setConstraintValue(2.3, i);
    }

    unsigned int number_joints, number_constraints;
    std::vector<double> some_joints, some_constraints;
    std::vector<double> lower_joint_limits, upper_joint_limits;
    std::vector<double> lower_constraint_limits, upper_constraint_limits;
    CBiRRTSpacePtr my_space;
};

TEST_F(CBiRRTSpaceTest, JointGettingSetting)
{
  ASSERT_TRUE(my_space);
  ASSERT_EQ(some_joints.size(), my_space->getNumberOfJoints());

  ompl::base::ScopedState<CBiRRTSpace> state(my_space);
  for(unsigned int i=0; i<some_joints.size(); i++)
  {
    state->setJointValue(some_joints[i], i);
    ASSERT_EQ(some_joints[i], state->getJointValue(i));
  }
}

TEST_F(CBiRRTSpaceTest, ConstraintGettingSetting)
{
  ASSERT_TRUE(my_space);
  ASSERT_EQ(some_constraints.size(), my_space->getNumberOfConstraints());

  ompl::base::ScopedState<CBiRRTSpace> state(my_space);
  for(unsigned int i=0; i<some_constraints.size(); i++)
  {
    state->setConstraintValue(some_constraints[i], i);
    ASSERT_EQ(some_constraints[i], state->getConstraintValue(i));
  }
}

TEST_F(CBiRRTSpaceTest, Boundaries)
{
  ASSERT_TRUE(my_space);
  ASSERT_EQ(lower_constraint_limits.size(), my_space->getNumberOfConstraints());
  ASSERT_EQ(upper_constraint_limits.size(), my_space->getNumberOfConstraints());
  ASSERT_EQ(lower_joint_limits.size(), my_space->getNumberOfJoints());
  ASSERT_EQ(upper_joint_limits.size(), my_space->getNumberOfJoints());

  ompl::base::RealVectorBounds joint_bounds(my_space->getNumberOfJoints());
  ompl::base::RealVectorBounds constraint_bounds(my_space->getNumberOfConstraints());

  joint_bounds.low = lower_joint_limits;
  joint_bounds.high = upper_joint_limits;
  constraint_bounds.low = lower_constraint_limits;
  constraint_bounds.high = upper_constraint_limits; 

  my_space->setBounds(joint_bounds, constraint_bounds);
  for (unsigned int i=0; i<my_space->getNumberOfJoints(); i++)
  {
    EXPECT_EQ(my_space->getJointBounds().low[i], lower_joint_limits[i]);
    EXPECT_EQ(my_space->getJointBounds().high[i], upper_joint_limits[i]);
  }
  for (unsigned int i=0; i<my_space->getNumberOfConstraints(); i++)
  {
    EXPECT_EQ(my_space->getConstraintBounds().low[i], lower_constraint_limits[i]);
    EXPECT_EQ(my_space->getConstraintBounds().high[i], upper_constraint_limits[i]);
  }

  EXPECT_GT(my_space->getMaximumExtent(), 0.0);
}

TEST_F(CBiRRTSpaceTest, SanityChecks)
{
  ASSERT_TRUE(my_space);
  my_space->sanityChecks();
}

TEST_F(CBiRRTSpaceTest, UpdateAndProjectConstraintValues)
{
  ASSERT_TRUE(my_space);
  ompl::base::State* some_state = my_space->allocState();
  CBiRRTSpace::StateType* cbirrt_state = some_state->as<CBiRRTSpace::StateType>();

  my_space->constraintProjectConfiguration(some_state);
  for(unsigned int i=0; i<my_space->getNumberOfConstraints(); i++)
    EXPECT_DOUBLE_EQ(2.3, cbirrt_state->getConstraintValue(i));

  my_space->freeState(some_state);
}

TEST_F(CBiRRTSpaceTest, SpaceInformation)
{
  ASSERT_TRUE(my_space);

  ASSERT_EQ(lower_constraint_limits.size(), my_space->getNumberOfConstraints());
  ASSERT_EQ(upper_constraint_limits.size(), my_space->getNumberOfConstraints());
  ASSERT_EQ(lower_joint_limits.size(), my_space->getNumberOfJoints());
  ASSERT_EQ(upper_joint_limits.size(), my_space->getNumberOfJoints());
  ompl::base::RealVectorBounds joint_bounds(my_space->getNumberOfJoints());
  ompl::base::RealVectorBounds constraint_bounds(my_space->getNumberOfConstraints());
  joint_bounds.low = lower_joint_limits;
  joint_bounds.high = upper_joint_limits;
  constraint_bounds.low = lower_constraint_limits;
  constraint_bounds.high = upper_constraint_limits; 
  my_space->setBounds(joint_bounds, constraint_bounds);
  ASSERT_GT(my_space->getMaximumExtent(), 0.0);

  ompl::base::SpaceInformation si(my_space);
  si.setStateValidityChecker(boost::bind(&CBiRRTSpaceTest::MyDummyIsValid, this, _1));
  si.setup();
  EXPECT_TRUE(si.isSetup());
}
