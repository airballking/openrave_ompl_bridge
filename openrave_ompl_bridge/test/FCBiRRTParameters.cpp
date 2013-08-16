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

TEST_F(FCBiRRTParametersTest, OwnMembers)
{
  Constraint::init();
  Feature oven_feature, bottle_feature;
  oven_feature.name = "oven_plane";
  oven_feature.pos = KDL::Vector(0.1, 0.2, 0.3);
  oven_feature.dir = KDL::Vector(0.0, 0.0, 1.0);

  bottle_feature.name = "bottle_axis";
  bottle_feature.pos = KDL::Vector(0.0, 0.0, 0.5);
  bottle_feature.dir = KDL::Vector(0.0, 0.0, 0.2);

  Constraint height_constraint, distance_constraint;
  height_constraint.name = "bottle height over oven";
  height_constraint.setFunction("height");
  height_constraint.tool_feature = bottle_feature;
  height_constraint.object_feature = oven_feature;

  distance_constraint.name = "distance of bottle from oven";
  distance_constraint.setFunction("distance");
  distance_constraint.tool_feature = bottle_feature;
  distance_constraint.object_feature = oven_feature;

  std::vector<Constraint> constraints, less_constraints;
  constraints.push_back(height_constraint);
  constraints.push_back(distance_constraint);
  less_constraints.push_back(height_constraint);

  Ranges commands, less_commands;
  commands.resize(2);
  less_commands.resize(1);
  for(unsigned int i=0; i<commands.size(); i++)
  {
    commands.pos_lo(i) = -0.1 - i;
    commands.pos_hi(i) = 0.1 + i;
    commands.weight(i) = 1.0;
    commands.max_vel(i) = commands.pos_hi(i) * commands.pos_hi(i);
    commands.min_vel(i) = 0.0;
  }
  for(unsigned int i=0; i<less_commands.size(); i++)
  {
    less_commands.pos_lo(i) = i + 0.1;
    less_commands.pos_hi(i) = i - 0.1;
    less_commands.weight(i) = 1.0;
    less_commands.max_vel(i) = 2.0;
    less_commands.min_vel(i) = -2.0;
  }

  parameters->goal_config = constraints;
  parameters->path_config = less_constraints;
  parameters->goal_command = commands;
  parameters->path_command = less_commands;

  std::vector<Constraint> constraint_query = parameters->GetGoalConfig();
  ASSERT_EQ(constraints.size(), constraint_query.size());
  for(unsigned int i=0; i<constraints.size(); i++)
    EXPECT_TRUE(Equal(constraints[i], constraint_query[i]));

  // see if it still works after typecasting because that involves serialization and de-serialization
  FCBiRRTParametersPtr pointer2(new FCBiRRTParameters());
  OpenRAVE::PlannerBase::PlannerParametersPtr base_pointer = boost::static_pointer_cast<OpenRAVE::PlannerBase::PlannerParameters>(parameters);
  pointer2->copy(base_pointer);

  constraint_query = pointer2->GetGoalConfig();
  ASSERT_EQ(constraints.size(), constraint_query.size());
  for(unsigned int i=0; i<constraints.size(); i++)
    EXPECT_TRUE(Equal(constraints[i], constraint_query[i]));

  constraint_query = pointer2->GetPathConfig();
  ASSERT_EQ(less_constraints.size(), constraint_query.size());
  for(unsigned int i=0; i<less_constraints.size(); i++)
    EXPECT_TRUE(Equal(less_constraints[i], constraint_query[i]));

  Ranges command_query = pointer2->GetGoalCommand();
  ASSERT_EQ(command_query.size(), commands.size());
  for(unsigned int i=0; i<commands.size(); i++)
    EXPECT_TRUE(Equal(commands, command_query));

  command_query = pointer2->GetPathCommand();
  ASSERT_EQ(less_commands.size(), command_query.size());
  for(unsigned int i=0; i<command_query.size(); i++)
    EXPECT_TRUE(Equal(command_query, less_commands));
}
