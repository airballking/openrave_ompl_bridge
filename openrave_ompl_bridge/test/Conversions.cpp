#include <gtest/gtest.h>
#include <openrave_ompl_bridge/Conversions.h>

using namespace openrave_ompl_bridge;

class ConversionsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      kdl_frame.p = KDL::Vector(0.1, 0.2, 0.3);
      kdl_frame.M = KDL::Rotation::RotY(1.2);
    }

    virtual void TearDown()
    {
    } 

    KDL::Frame kdl_frame;
};

TEST_F(ConversionsTest, Frame)
{
  OpenRAVE::Transform or_transform = toOR(kdl_frame);
  KDL::Frame kdl_frame2 = toKDL(or_transform);  
  EXPECT_DOUBLE_EQ(kdl_frame.p.x(), kdl_frame2.p.x());
  EXPECT_DOUBLE_EQ(kdl_frame.p.y(), kdl_frame2.p.y());
  EXPECT_DOUBLE_EQ(kdl_frame.p.z(), kdl_frame2.p.z());

  double x, y, z, w, x2, y2, z2, w2;
  kdl_frame.M.GetQuaternion(x, y, z, w);
  kdl_frame2.M.GetQuaternion(x2, y2, z2, w2);
  EXPECT_DOUBLE_EQ(x, x2);
  EXPECT_DOUBLE_EQ(y, y2);
  EXPECT_DOUBLE_EQ(z, z2);
  EXPECT_DOUBLE_EQ(w, w2);
}

TEST_F(ConversionsTest, Jacobian)
{
  KDL::Jacobian jac, jac2;
  jac.resize(3);
  jac2.resize(3);
  for(unsigned int i=0; i<jac.rows(); i++)
  {
    for(unsigned int j=0; j<jac.columns(); j++)
    {
      jac(i,j) = (double)j + ((double)i)*0.1;
    }
  }

  std::vector<double> pos_jac, ang_jac;
  for(unsigned int i=0; i<jac.rows(); i++)
  {
    for(unsigned int j=0; j<jac.columns(); j++)
    {
      if(i<3)
      {
        pos_jac.push_back((double)j + ((double)i)*0.1);
      }
      else
      {
        ang_jac.push_back((double)j + ((double)i)*0.1);
      }
    }
  }

  toKDL(jac2, pos_jac, ang_jac);
  for(unsigned int i=0; i<jac.rows(); i++)
  {
    for(unsigned int j=0; j<jac.columns(); j++)
    {
      EXPECT_DOUBLE_EQ(jac(i,j), jac2(i,j));
    }
  }  
}

TEST_F(ConversionsTest, Rotation)
{
  EXPECT_TRUE(Equal(KDL::Rotation::RotX(M_PI/3.0), toKDL(toOR(KDL::Frame(KDL::Rotation::RotX(M_PI/3.0), KDL::Vector(0.0, 0.0, 0.0)))).M));
  EXPECT_TRUE(Equal(KDL::Rotation::RotY(M_PI/3.0), toKDL(toOR(KDL::Frame(KDL::Rotation::RotY(M_PI/3.0), KDL::Vector(0.0, 0.0, 0.0)))).M));
  EXPECT_TRUE(Equal(KDL::Rotation::RotZ(M_PI/3.0), toKDL(toOR(KDL::Frame(KDL::Rotation::RotZ(M_PI/3.0), KDL::Vector(0.0, 0.0, 0.0)))).M));
  EXPECT_TRUE(Equal(KDL::Rotation::RotX(-M_PI/3.0), toKDL(toOR(KDL::Frame(KDL::Rotation::RotX(-M_PI/3.0), KDL::Vector(0.0, 0.0, 0.0)))).M));
  EXPECT_TRUE(Equal(KDL::Rotation::RotY(-M_PI/3.0), toKDL(toOR(KDL::Frame(KDL::Rotation::RotY(-M_PI/3.0), KDL::Vector(0.0, 0.0, 0.0)))).M));
  EXPECT_TRUE(Equal(KDL::Rotation::RotZ(-M_PI/3.0), toKDL(toOR(KDL::Frame(KDL::Rotation::RotZ(-M_PI/3.0), KDL::Vector(0.0, 0.0, 0.0)))).M));
  EXPECT_TRUE(Equal(KDL::Rotation::Identity(), KDL::Rotation::Identity()));
  EXPECT_TRUE(Equal(KDL::Rotation::Identity(), toKDL(OpenRAVE::Transform()).M));
}
