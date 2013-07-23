#include <gtest/gtest.h>

#include <openrave-core.h>
#include <openrave_ompl_bridge/OMPLPlannerRRTConnect.h>

TEST(PluginTests, Constructor)
{
  OpenRAVE::RaveInitialize();
  OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
  // we can directly construct the planner object
  openrave_ompl_bridge::OMPLPlannerRRTConnect planner(env);  
  // or ask OpenRAVE to do it in its factory
  OpenRAVE::PlannerBasePtr planner_pointer = OpenRAVE::RaveCreatePlanner(env, "omplrrtconnect");  
  // casting using boost should then be working
  openrave_ompl_bridge::OMPLPlannerRRTConnectPtr planner_pointer2 = boost::static_pointer_cast<openrave_ompl_bridge::OMPLPlannerRRTConnect>(planner_pointer);  

  EXPECT_TRUE(planner_pointer);
  EXPECT_TRUE(planner_pointer2);
  OpenRAVE::RaveDestroy();
}
