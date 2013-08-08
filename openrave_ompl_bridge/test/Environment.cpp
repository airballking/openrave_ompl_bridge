#include <gtest/gtest.h>
#include <openrave-core.h>
#include <openrave_ompl_bridge/Environment.h>
#include <openrave_ompl_bridge/Conversions.h>
#include <vector>

using namespace openrave_ompl_bridge;

class EnvironmentTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      OpenRAVE::RaveInitialize();
      env = OpenRAVE::RaveCreateEnvironment();
      test_env = EnvironmentPtr(new Environment(env));

      // creating body 'left'
      OpenRAVE::KinBodyPtr body = OpenRAVE::RaveCreateKinBody(env, "");
      std::vector<OpenRAVE::AABB> boxes;
      OpenRAVE::AABB box(OpenRAVE::Vector(0.0, 0.0, 0.0), OpenRAVE::Vector(0.1, 0.1, 0.1));
      boxes.push_back(box);
      body->InitFromBoxes(boxes, true);
      body->SetName("left");
      OpenRAVE::Transform t;
      t.trans = OpenRAVE::Vector(0.0, -1.0, 1.0);
      body->SetTransform(t);
      env->Add(body);

      // creating body 'right_1'
      body = OpenRAVE::RaveCreateKinBody(env, "");
      body->InitFromBoxes(boxes, true);
      body->SetName("right_1");
      t.trans = OpenRAVE::Vector(0.0, 1.0, 0.5);
      body->SetTransform(t);
      env->Add(body);

      // creating body 'right_2'
      body = OpenRAVE::RaveCreateKinBody(env, "");
      body->InitFromBoxes(boxes, true);
      body->SetName("right_2");
      toOR(t, KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(-0.2, 0.0, 0.5)));
      body->SetTransform(t);
      env->Add(body);
    }

    virtual void TearDown()
    {
      OpenRAVE::RaveDestroy();
    }

    OpenRAVE::EnvironmentBasePtr env;
    EnvironmentPtr test_env;
};

TEST_F(EnvironmentTest, GetObjectFrameInWorld)
{
  KDL::Frame left_frame = test_env->getObjectFrameInWorld("left");
  EXPECT_DOUBLE_EQ(left_frame.p[0], 0.0);
  EXPECT_DOUBLE_EQ(left_frame.p[1], -1.0);
  EXPECT_DOUBLE_EQ(left_frame.p[2], 1.0);
  EXPECT_TRUE(Equal(left_frame.M, KDL::Rotation::Identity()));

  KDL::Frame r1_frame = test_env->getObjectFrameInWorld("right_1");
  EXPECT_DOUBLE_EQ(r1_frame.p[0], 0.0);
  EXPECT_DOUBLE_EQ(r1_frame.p[1], 1.0);
  EXPECT_DOUBLE_EQ(r1_frame.p[2], 0.5);
  EXPECT_TRUE(Equal(r1_frame.M, KDL::Rotation::Identity()));

  KDL::Frame r2_frame = test_env->getObjectFrameInWorld("right_2");
  EXPECT_DOUBLE_EQ(r2_frame.p[0], -0.2);
  EXPECT_DOUBLE_EQ(r2_frame.p[1], 0.0);
  EXPECT_DOUBLE_EQ(r2_frame.p[2], 0.5);
  EXPECT_TRUE(Equal(r2_frame.M, KDL::Rotation::RotZ(M_PI/2.0)));
} 

TEST_F(EnvironmentTest, GetTransform)
{
  KDL::Frame left_to_right1 = test_env->getTransform("left", "right_1");
  EXPECT_DOUBLE_EQ(left_to_right1.p[0], 0.0);
  EXPECT_DOUBLE_EQ(left_to_right1.p[1], 2.0);
  EXPECT_DOUBLE_EQ(left_to_right1.p[2], -0.5);
  EXPECT_TRUE(Equal(left_to_right1.M, KDL::Rotation::Identity()));

  KDL::Frame right1_to_left = test_env->getTransform("right_1", "left");
  EXPECT_DOUBLE_EQ(right1_to_left.p[0], 0.0);
  EXPECT_DOUBLE_EQ(right1_to_left.p[1], -2.0);
  EXPECT_DOUBLE_EQ(right1_to_left.p[2], 0.5);
  EXPECT_TRUE(Equal(right1_to_left.M, KDL::Rotation::Identity()));

  KDL::Frame left_to_right2 = test_env->getTransform("left", "right_2");
  EXPECT_DOUBLE_EQ(left_to_right2.p[0], -0.2);
  EXPECT_DOUBLE_EQ(left_to_right2.p[1], 1.0);
  EXPECT_DOUBLE_EQ(left_to_right2.p[2], -0.5);
  EXPECT_TRUE(Equal(left_to_right2.M, KDL::Rotation::RotZ(M_PI/2.0)));
}
