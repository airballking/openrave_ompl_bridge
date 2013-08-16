#include <gtest/gtest.h>
#include <openrave_ompl_bridge/YamlUtils.h>

#include <sstream>

using namespace openrave_ompl_bridge;

class YamlUtilsTest : public ::testing::Test 
{
  protected:
    virtual void SetUp()
    {
      Constraint::init();

      frame.p = KDL::Vector(1.0, 2.0, 3.0);

      oven_feature.name = "oven_plane";
      oven_feature.pos = KDL::Vector(0.1, 0.2, 0.3);
      oven_feature.dir = KDL::Vector(0.0, 0.0, 1.0);

      bottle_feature.name = "bottle_axis";
      bottle_feature.pos = KDL::Vector(0.0, 0.0, 0.5);
      bottle_feature.dir = KDL::Vector(0.0, 0.0, 0.2);
 
      height_constraint.name = "bottle height over oven";
      height_constraint.setFunction("height");
      height_constraint.tool_feature = bottle_feature;
      height_constraint.object_feature = oven_feature;

      distance_constraint.name = "distance of bottle from oven";
      distance_constraint.setFunction("distance");
      distance_constraint.tool_feature = bottle_feature;
      distance_constraint.object_feature = oven_feature;

      constraints.clear();
      constraints.push_back(height_constraint);
      constraints.push_back(distance_constraint);
 
      joints.resize(7);
      for(unsigned int i=0; i<joints.rows(); i++)
        joints(i) = 0.1*i;

      commands.resize(3);
      for(unsigned int i=0; i<commands.size(); i++)
      {
        commands.pos_lo(i) = -0.1 - i;
        commands.pos_hi(i) = 0.1 + i;
        commands.weight(i) = 1.0;
        commands.max_vel(i) = commands.pos_hi(i) * commands.pos_hi(i);
        commands.min_vel(i) = 0.0;
      }
    }

    virtual void TearDown()
    {
    } 

    KDL::Frame frame;
    KDL::JntArray joints;
    Feature oven_feature, bottle_feature;
    Constraint height_constraint, distance_constraint;
    std::vector<Constraint> constraints;
    Ranges commands;
};

TEST_F(YamlUtilsTest, KDLVector)
{
  // writing it out
  YAML::Emitter out;
  out << frame.p;

  // reading it back
  KDL::Vector v;
  std::istringstream iss;
  iss.str (out.c_str());

  YAML::Parser parser(iss);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc >> v;
  }

  // making sure everything checks out
  EXPECT_TRUE(Equal(frame.p, v));
}

TEST_F(YamlUtilsTest, KDLRotation)
{
  // writing it out
  YAML::Emitter out;
  out << frame.M;

  // reading it back
  KDL::Rotation r;
  std::istringstream iss;
  iss.str (out.c_str());

  YAML::Parser parser(iss);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc >> r;
  }

  // making sure everything checks out
  EXPECT_TRUE(Equal(frame.M, r));
}

TEST_F(YamlUtilsTest, KDLFrame)
{
  // writing it out
  YAML::Emitter out;
  out << frame;

  // reading it back
  KDL::Frame f;
  std::istringstream iss;
  iss.str (out.c_str());

  YAML::Parser parser(iss);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc >> f;
  }

  // making sure everything checks out
  EXPECT_TRUE(Equal(frame, f));
}

TEST_F(YamlUtilsTest, Feature)
{
  // writing it out
  YAML::Emitter out;
  out << oven_feature;

  // reading it back
  Feature f;
  std::istringstream iss;
  iss.str (out.c_str());

  YAML::Parser parser(iss);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc >> f;
  }

  // making sure everything checks out
  EXPECT_TRUE(Equal(oven_feature, f));
}

TEST_F(YamlUtilsTest, Constraint)
{
  // writing it out
  YAML::Emitter out;
  out << height_constraint;

  // reading it back
  Constraint c;
  std::istringstream iss;
  iss.str (out.c_str());

  YAML::Parser parser(iss);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc >> c;
  }

  // making sure everything checks out
  EXPECT_TRUE(Equal(height_constraint, c));
}

TEST_F(YamlUtilsTest, JntArray)
{
  // writing it out
  YAML::Emitter out;
  out << joints;

  // reading it back
  KDL::JntArray j;
  std::istringstream iss;
  iss.str (out.c_str());

  YAML::Parser parser(iss);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc >> j;
  }

  // making sure everything checks out
  EXPECT_TRUE(Equal(joints, j));
}

TEST_F(YamlUtilsTest, Ranges)
{
  // writing it out
  YAML::Emitter out;
  out << commands;

  // reading it back
  Ranges r;
  std::istringstream iss;
  iss.str (out.c_str());

  YAML::Parser parser(iss);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc >> r;
  }

  // making sure everything checks out
  EXPECT_TRUE(Equal(commands, r));
}

TEST_F(YamlUtilsTest, VectorOfConstraints)
{
  // writing it out
  YAML::Emitter out;
  out << constraints;

  // reading it back
  std::vector<Constraint> c;
  std::istringstream iss;
  iss.str (out.c_str());

  YAML::Parser parser(iss);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc >> c;
  }

  // making sure everything checks out
  ASSERT_EQ(constraints.size(), c.size());
  for(unsigned int i=0; i<c.size(); i++)
    EXPECT_TRUE(Equal(constraints[i], c[i]));
}
