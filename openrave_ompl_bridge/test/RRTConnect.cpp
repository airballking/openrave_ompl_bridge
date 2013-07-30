#include <gtest/gtest.h>

#include <vector>

#include <openrave-core.h>
#include <openrave_ompl_bridge/RRTConnect.h>
#include <openrave_ompl_bridge/RRTConnectParameters.h>

using namespace openrave_ompl_bridge;

class PlannerTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      OpenRAVE::RaveInitialize();
      env = OpenRAVE::RaveCreateEnvironment();

      planner = OpenRAVE::RaveCreatePlanner(env, "omplrrtconnect");
      planner2 = boost::static_pointer_cast<openrave_ompl_bridge::RRTConnect>(planner);

      herb = env->ReadRobotXMLFile("robots/herb2_padded_nosensors.robot.xml");
      env->Add(herb);

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
      }

      parameters = RRTConnectParametersPtr(new RRTConnectParameters());

      herb->SetActiveDOFs(right_arm->GetArmIndices());
      parameters->SetRobotActiveJoints(herb);

      parameters->timelimit = 3.0;
      parameters->smoothing_timelimit = 1.0;

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

    virtual void TearDown()
    {
      OpenRAVE::RaveDestroy();
    }

    OpenRAVE::EnvironmentBasePtr env;
    OpenRAVE::RobotBasePtr herb; 
    OpenRAVE::RobotBase::ManipulatorPtr right_arm, left_arm;
    OpenRAVE::PlannerBasePtr planner;
    RRTConnectPtr planner2;
    RRTConnectParametersPtr parameters;
};

TEST_F(PlannerTest, InitPlan)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(parameters);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  EXPECT_TRUE(planner2->InitPlan(herb, parameters));
}

TEST_F(PlannerTest, PlanPath)
{
  ASSERT_TRUE(planner2);
  ASSERT_TRUE(herb);
  ASSERT_TRUE(right_arm);
  ASSERT_TRUE(parameters);
  herb->SetActiveDOFs(right_arm->GetArmIndices());
  ASSERT_TRUE(planner2->InitPlan(herb, parameters));
  OpenRAVE::TrajectoryBasePtr trajectory = OpenRAVE::RaveCreateTrajectory(env, right_arm->GetArmIndices().size());
  EXPECT_EQ(OpenRAVE::PS_HasSolution, planner2->PlanPath(trajectory));
}
