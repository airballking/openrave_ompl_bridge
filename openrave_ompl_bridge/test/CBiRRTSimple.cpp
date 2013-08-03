#include <gtest/gtest.h>

#include <openrave_ompl_bridge/CBiRRTSpace.h>
#include <openrave_ompl_bridge/CBiRRTMotionValidator.h>
#include <ompl/geometric/SimpleSetup.h>

using namespace openrave_ompl_bridge;

class CBiRRTSimpleTest : public ::testing::Test 
{
  public:
    bool checkPathConstraints(const ompl::base::State *state) 
    {
      assert(state);
      return true;
    }

    void projection(ompl::base::State *state) 
    {
      assert(state);
      assert(num_joints==num_constraints);

      for(unsigned int i=0; i<num_joints; i++)
        state->as<CBiRRTSpace::StateType>()->setConstraintValue(state->as<CBiRRTSpace::StateType>()->getJointValue(i), i);
    }

    bool isValid(const ompl::base::State *state) 
    {
      return true;
    }

 protected:
    virtual void SetUp()
    {
      num_joints = 7;
      num_constraints = num_joints;
    }

    virtual void TearDown()
    {
    }

  unsigned int num_joints, num_constraints;
};

TEST_F(CBiRRTSimpleTest, )
{
  // the space
  ompl::base::StateSpacePtr space(new CBiRRTSpace(num_joints, num_constraints));

  // primitive case where constraint-space equals joint-space
  ompl::base::RealVectorBounds bounds(num_joints);
  bounds.setLow(-1.0);
  bounds.setHigh(1.0);
  space->as<CBiRRTSpace>()->setBounds(bounds, bounds);
// setting the function hooks
  space->as<CBiRRTSpace>()->setConstraintProjectionFunction(boost::bind(&CBiRRTSimpleTest::projection, this, _1));
  space->as<CBiRRTSpace>()->setPathConstraintsCheckFunction(boost::bind(&CBiRRTSimpleTest::checkPathConstraints, this, _1));

  // creating the setup
  ompl::geometric::SimpleSetup setup(space);
  
  // setting the validity checker
  setup.setStateValidityChecker(boost::bind(&CBiRRTSimpleTest::isValid, this, _1));

  // some random start and goal states
  ompl::base::ScopedState<> start(space);
  ompl::base::ScopedState<> goal(space);
  start.random();
  goal.random();
  setup.setStartAndGoalStates(start, goal);
  ASSERT_TRUE(setup.solve(1.0)); 
  ASSERT_TRUE(setup.haveSolutionPath());
  setup.simplifySolution(1.0);
}
