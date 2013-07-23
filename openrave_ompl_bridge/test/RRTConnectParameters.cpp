#include <gtest/gtest.h>

#include <vector>

#include <openrave-core.h>
#include <openrave_ompl_bridge/OMPLPlannerParametersRRTConnect.h>

using namespace openrave_ompl_bridge;

class ParameterTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      OpenRAVE::RaveInitialize();
      parameters = OMPLPlannerParametersRRTConnectPtr(new OMPLPlannerParametersRRTConnect());
      no_timeout = 1.0;
      some_timeout = 2.31;

      empty_joints.clear();
      full_joints.clear();

      for (unsigned int i=0; i<7; i++)
      {
        full_joints.push_back(1.0);
      }
    }

    virtual void TearDown()
    {
      OpenRAVE::RaveDestroy();
    }

    OMPLPlannerParametersRRTConnectPtr parameters;
    std::vector<double> empty_joints, full_joints;
    double no_timeout, some_timeout;

};

TEST_F(ParameterTest, GetGoalConfiguration)
{
  parameters->vgoalconfig = empty_joints;
  EXPECT_TRUE(empty_joints.size() == parameters->GetGoalConfiguration().size());
  
  parameters->vgoalconfig = full_joints;
  EXPECT_TRUE(full_joints.size() == parameters->GetGoalConfiguration().size());
  for(unsigned int i=0; i<full_joints.size(); i++)
    EXPECT_TRUE(parameters->GetGoalConfiguration()[i] == full_joints[i]);
}

TEST_F(ParameterTest, GetStartConfiguration)
{
  parameters->vinitialconfig = empty_joints;
  EXPECT_TRUE(empty_joints.size() == parameters->GetStartConfiguration().size());
  
  parameters->vinitialconfig = full_joints;
  EXPECT_TRUE(full_joints.size() == parameters->GetStartConfiguration().size());
  for(unsigned int i=0; i<full_joints.size(); i++)
    EXPECT_TRUE(parameters->GetStartConfiguration()[i] == full_joints[i]);
}

TEST_F(ParameterTest, GetTimeLimit)
{
  parameters->timelimit=no_timeout;
  EXPECT_TRUE(no_timeout == parameters->GetTimeLimit());

  parameters->timelimit=some_timeout;
  EXPECT_TRUE(some_timeout == parameters->GetTimeLimit());
}
