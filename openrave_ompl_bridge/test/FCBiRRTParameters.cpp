#include <gtest/gtest.h>

#include <vector>

#include <openrave-core.h>
#include <openrave_ompl_bridge/RRTConnectParameters.h>
#include <openrave_ompl_bridge/FCBiRRTParameters.h>

using namespace openrave_ompl_bridge;

class FCBiRRTParametersTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      OpenRAVE::RaveInitialize();
      parameters = FCBiRRTParametersPtr(new FCBiRRTParameters());
      some_timeout = 2.31;
      some_timeout2 = 3.75;
    }

    virtual void TearDown()
    {
      OpenRAVE::RaveDestroy();
    }

    FCBiRRTParametersPtr parameters;
    double some_timeout, some_timeout2;
};

TEST_F(FCBiRRTParametersTest, ParentMembers)
{
  parameters->planning_timelimit = some_timeout;
  parameters->smoothing_timelimit = some_timeout2;

  EXPECT_DOUBLE_EQ(parameters->GetPlanningTimeLimit(), some_timeout);
  EXPECT_DOUBLE_EQ(parameters->GetSmoothingTimeLimit(), some_timeout2);
} 


TEST_F(FCBiRRTParametersTest, CopyTypeCast)
{
  parameters->planning_timelimit = some_timeout;
  parameters->smoothing_timelimit = some_timeout2;

  FCBiRRTParametersPtr pointer2(new FCBiRRTParameters());
  OpenRAVE::PlannerBase::PlannerParametersPtr base_pointer = boost::static_pointer_cast<OpenRAVE::PlannerBase::PlannerParameters>(parameters);

  pointer2->copy(base_pointer);
  EXPECT_EQ(some_timeout, pointer2->GetPlanningTimeLimit());
  EXPECT_EQ(some_timeout2, pointer2->GetSmoothingTimeLimit());
}

