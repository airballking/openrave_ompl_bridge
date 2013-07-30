#include <gtest/gtest.h>

#include <vector>

#include <openrave-core.h>
#include <openrave_ompl_bridge/RRTConnectParameters.h>

using namespace openrave_ompl_bridge;

class ParameterTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      OpenRAVE::RaveInitialize();
      parameters = RRTConnectParametersPtr(new RRTConnectParameters());
      no_timeout = 1.0;
      some_timeout = 2.31;
      some_timeout2 = 3.75;

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

    RRTConnectParametersPtr parameters;
    std::vector<double> empty_joints, full_joints;
    double no_timeout, some_timeout, some_timeout2;

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

TEST_F(ParameterTest, GetTimeLimits)
{
  parameters->timelimit=no_timeout;
  parameters->smoothing_timelimit=some_timeout;
  EXPECT_EQ(no_timeout, parameters->GetTimeLimit());
  EXPECT_EQ(some_timeout, parameters->GetSmoothingTimeLimit());

  parameters->timelimit=some_timeout;
  parameters->smoothing_timelimit=some_timeout2;
  EXPECT_EQ(some_timeout, parameters->GetTimeLimit());
  EXPECT_EQ(some_timeout2, parameters->GetSmoothingTimeLimit());
}

TEST_F(ParameterTest, CopyTypeCast)
{
  parameters->timelimit = some_timeout;
  parameters->smoothing_timelimit = some_timeout2;

  RRTConnectParametersPtr pointer2(new RRTConnectParameters());
  OpenRAVE::PlannerBase::PlannerParametersPtr base_pointer = boost::static_pointer_cast<OpenRAVE::PlannerBase::PlannerParameters>(parameters);

  pointer2->copy(base_pointer);
  EXPECT_EQ(some_timeout, pointer2->GetTimeLimit());
  EXPECT_EQ(some_timeout2, pointer2->GetSmoothingTimeLimit());
}

