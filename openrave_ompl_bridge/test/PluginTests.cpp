#include <gtest/gtest.h>

#include <openrave-core.h>
#include <openrave_ompl_bridge/OMPLPlannerRRTConnect.h>

TEST(PluginTests, Constructor)
{
  OpenRAVE::RaveInitialize();
  OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
  openrave_ompl_bridge::OMPLPlannerRRTConnect planner(env);  
  EXPECT_TRUE(true);
}
