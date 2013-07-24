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
      GetRaveEnvironment();     
      CreatePlanner();
      LoadRobot();
      GetManipulators();    
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
