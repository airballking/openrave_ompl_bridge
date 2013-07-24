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
      if(env)
        robot = env->ReadRobotXMLFile("robots/herb2_padded_nosensors.robot.xml");
    }

    virtual void TearDown()
    {
      OpenRAVE::RaveDestroy();
    }

    OpenRAVE::EnvironmentBasePtr env;
    OpenRAVE::RobotBasePtr robot;
    OpenRAVE::PlannerBasePtr planner;
    OMPLPlannerRRTConnectPtr planner2;
};

TEST_F(PlannerTest, FixtureSetup)
{
  EXPECT_TRUE(env);
  EXPECT_TRUE(planner);
  EXPECT_TRUE(planner2);
  EXPECT_TRUE(robot);
}
