#include <gtest/gtest.h>

#include <vector>

#include <openrave-core.h>
#include <openrave_ompl_bridge/OMPLPlannerRRTConnect.h>

using namespace openrave_ompl_bridge;

class PlannerTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      OpenRAVE::RaveInitialize();
      env = OpenRAVE::RaveCreateEnvironment();
      planner = OpenRAVE::RaveCreatePlanner(env, "omplrrtconnect");
      planner2 = boost::static_pointer_cast<openrave_ompl_bridge::OMPLPlannerRRTConnect>(planner);
    }

    virtual void TearDown()
    {
      OpenRAVE::RaveDestroy();
    }

    OpenRAVE::EnvironmentBasePtr env;
    OpenRAVE::PlannerBasePtr planner;
    OMPLPlannerRRTConnectPtr planner2;
};

TEST_F(PlannerTest, FixtureSetup)
{
  EXPECT_TRUE(env);
  EXPECT_TRUE(planner);
  EXPECT_TRUE(planner2);
}
