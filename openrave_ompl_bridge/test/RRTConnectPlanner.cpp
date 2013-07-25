#include <gtest/gtest.h>

#include <vector>

#include <openrave-core.h>
#include <openrave_ompl_bridge/OMPLPlannerRRTConnect.h>
#include <openrave_ompl_bridge/OMPLPlannerParametersRRTConnect.h>

using namespace openrave_ompl_bridge;

class PlannerTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      GetRaveEnvironment();
      CreatePlanner();
      LoadRobot();
      GetManipulators();    
      CreateParameters();
    }

    void GetRaveEnvironment()
    {
      OpenRAVE::RaveInitialize();
      env = OpenRAVE::RaveCreateEnvironment();

    }

    void CreatePlanner()
    {
      if(env)
      {
        planner = OpenRAVE::RaveCreatePlanner(env, "omplrrtconnect");
        planner2 = boost::static_pointer_cast<openrave_ompl_bridge::OMPLPlannerRRTConnect>(planner);
      }
    }

    void CreateParameters()
    {
      parameters = OMPLPlannerParametersRRTConnectPtr(new OMPLPlannerParametersRRTConnect());

      herb->SetActiveDOFs(right_arm->GetArmIndices());
      parameters->SetRobotActiveJoints(herb);

      parameters->timelimit = 5.0;

      parameters->vinitialconfig.clear();
      herb->GetActiveDOFValues(parameters->vinitialconfig);

      parameters->vgoalconfig.clear();
      for(unsigned int i=0; i<herb->GetActiveDOF(); i++)
        parameters->vgoalconfig.push_back(1.0);

      std::vector<int> joint_indices;
      for (unsigned int i=0; i<herb->GetJoints().size(); i++)
        joint_indices.push_back((int)i);
      herb->SetActiveDOFs(joint_indices);

    }

    void LoadRobot()
    {
      if(env)
      {
        herb = env->ReadRobotXMLFile("robots/herb2_padded_nosensors.robot.xml");
        env->Add(herb);
      }
    }
     
    void GetManipulators()
    {
      if(herb)
      {
        std::vector<OpenRAVE::RobotBase::ManipulatorPtr> herb_manipulators = herb->GetManipulators();

        for (unsigned int i=0; i<herb_manipulators.size(); i++)
        {
          if(herb_manipulators[i] && (herb_manipulators[i]->GetName().compare("right_wam") == 0))
          {
            right_arm = herb_manipulators[i];
          }
          else if(herb_manipulators[i] && (herb_manipulators[i]->GetName().compare("left_wam") == 0))
          {
            left_arm = herb_manipulators[i];
          }
          else if(herb_manipulators[i] && (herb_manipulators[i]->GetName().compare("head_wam") == 0))
          {
            head = herb_manipulators[i];
          }
        }
      }
   }

    virtual void TearDown()
    {
      OpenRAVE::RaveDestroy();
    }

    OpenRAVE::EnvironmentBasePtr env;
    OpenRAVE::RobotBasePtr herb; 
    OpenRAVE::RobotBase::ManipulatorPtr right_arm, left_arm, head;
    OpenRAVE::PlannerBasePtr planner;
    OMPLPlannerRRTConnectPtr planner2;
    OMPLPlannerParametersRRTConnectPtr parameters;
};

TEST_F(PlannerTest, CopyRobot)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  EXPECT_TRUE(planner2->CopyRobot(herb));
}

TEST_F(PlannerTest, GetRobotDOF)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(head);
  ASSERT_TRUE(herb);

  EXPECT_EQ(0, planner2->GetRobotDOF());  

  ASSERT_TRUE(planner2->CopyRobot(herb));
  EXPECT_EQ(24, planner2->GetRobotDOF());

  herb->SetActiveDOFs(head->GetArmIndices());
  EXPECT_EQ(2, planner2->GetRobotDOF());

  std::vector<int> no_joints;
  herb->SetActiveDOFs(no_joints);
  EXPECT_EQ(0, planner2->GetRobotDOF());

  herb->SetActiveDOFs(right_arm->GetArmIndices());
  EXPECT_EQ(7, planner2->GetRobotDOF());
}

TEST_F(PlannerTest, HasActiveRobotDOF)
{
  ASSERT_TRUE(planner2);
  EXPECT_FALSE(planner2->HasActiveRobotDOF());

  ASSERT_TRUE(herb);
  ASSERT_TRUE(planner2->CopyRobot(herb));
  EXPECT_TRUE(planner2->HasActiveRobotDOF());

  ASSERT_TRUE(left_arm);
  herb->SetActiveDOFs(left_arm->GetArmIndices());
  EXPECT_TRUE(planner2->HasActiveRobotDOF());
  
  std::vector<int> no_joints;
  herb->SetActiveDOFs(no_joints);
  EXPECT_FALSE(planner2->HasActiveRobotDOF());
}

TEST_F(PlannerTest, GetRobotActiveJointLimits)
{
  ASSERT_TRUE(planner2);
  std::vector<double> lower, upper;
  EXPECT_FALSE(planner2->GetRobotActiveJointLimits(lower, upper));

  ASSERT_TRUE(herb);
  ASSERT_TRUE(planner2->CopyRobot(herb));
  EXPECT_TRUE(planner2->GetRobotActiveJointLimits(lower, upper));
  EXPECT_EQ(lower.size(), planner2->GetRobotDOF());
  EXPECT_EQ(lower.size(), upper.size());

  ASSERT_TRUE(right_arm);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  EXPECT_TRUE(planner2->GetRobotActiveJointLimits(lower, upper));
  EXPECT_EQ(lower.size(), planner2->GetRobotDOF());
  EXPECT_EQ(lower.size(), upper.size());
}

TEST_F(PlannerTest, CheckForRobotCollisions)
{
  double no_col[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double col[] = {4.8, 0.0, 0.0, 2.0, -2.0, 0.0, 0.0};
  double left[] = {0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> no_collision_config (no_col, no_col + sizeof(no_col) / sizeof(no_col[0]) );
  std::vector<double> collision_config (col, col + sizeof(col) / sizeof(col[0]) );
  std::vector<double> left_arm_config (left, left + sizeof(left) / sizeof(left[0]) );

  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(planner2->CopyRobot(herb));
  ASSERT_TRUE(left_arm);
  herb->SetActiveDOFs(left_arm->GetArmIndices());
  herb->SetActiveDOFValues(left_arm_config); 
  ASSERT_TRUE(right_arm);
  herb->SetActiveDOFs(right_arm->GetArmIndices());

  EXPECT_FALSE(planner2->IsActiveRobotConfigurationInCollision(no_collision_config));
  EXPECT_TRUE(planner2->IsActiveRobotConfigurationInCollision(collision_config));
}

TEST_F(PlannerTest, CopyParameters)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(parameters);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  EXPECT_TRUE(planner2->CopyParameters(parameters));
}

TEST_F(PlannerTest, ResetStateSpaceDimensions)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(parameters);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  ASSERT_TRUE(planner2->CopyRobot(herb));
  ASSERT_TRUE(planner2->CopyParameters(parameters));
  EXPECT_TRUE(planner2->ResetStateSpaceDimensions());
}

TEST_F(PlannerTest, ResetStateSpaceBoundaries)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(parameters);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  ASSERT_TRUE(planner2->CopyRobot(herb));
  ASSERT_TRUE(planner2->CopyParameters(parameters));
  ASSERT_TRUE(planner2->ResetStateSpaceDimensions());
  ASSERT_EQ(herb->GetActiveDOF(), parameters->GetStartConfiguration().size());
  ASSERT_EQ(herb->GetActiveDOF(), parameters->GetGoalConfiguration().size());
  ASSERT_EQ(herb->GetActiveDOF(), planner2->GetStateSpaceDimensions());
  ASSERT_EQ(7, parameters->GetStartConfiguration().size());
  ASSERT_EQ(7, parameters->GetGoalConfiguration().size());
  ASSERT_EQ(7, planner2->GetStateSpaceDimensions());
  EXPECT_TRUE(planner2->ResetStateSpaceBoundaries());
}

TEST_F(PlannerTest, ResetSimpleSetup)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(parameters);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  ASSERT_TRUE(planner2->CopyRobot(herb));
  ASSERT_TRUE(planner2->CopyParameters(parameters));
  ASSERT_TRUE(planner2->ResetStateSpaceDimensions());
  ASSERT_TRUE(planner2->ResetStateSpaceBoundaries());
  planner2->ResetSimpleSetup();
  EXPECT_TRUE(true);
}

TEST_F(PlannerTest, InitPlan)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(parameters);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  EXPECT_TRUE(planner2->InitPlan(herb, parameters));
}
