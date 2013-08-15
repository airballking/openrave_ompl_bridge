#ifndef OPENRAVE_OMPL_BRIDGE_YAML_UTILS_H
#define OPENRAVE_OMPL_BRIDGE_YAML_UTILS_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Controller.h>

#include "yaml-cpp/yaml.h"

namespace openrave_ompl_bridge
{
  YAML::Emitter& operator << (YAML::Emitter& out, const KDL::Vector& v)
  {
    out << YAML::Flow;
    out << YAML::BeginSeq << v.x() << v.y() << v.z() << YAML::EndSeq;
    return out;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const KDL::Rotation& r)
  {
    double x,y,z,w;
    r.GetQuaternion(x, y, z, w);
    out << YAML::Flow;
    out << YAML::BeginSeq << x << y << z << w << YAML::EndSeq;
    return out;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const KDL::Frame& f)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "translation" << YAML::Value << f.p;
    out << YAML::Key << "orientation" << YAML::Value << f.M; 
    out << YAML::EndMap;
    return out;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const KDL::JntArray& j)
  {
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for(unsigned int i=0; i<j.rows(); i++)
      out << j(i);    

    out << YAML::EndSeq;
    return out;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const Feature& f)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << f.name;
    out << YAML::Key << "position" << YAML::Value << f.pos; 
    out << YAML::Key << "direction" << YAML::Value << f.dir; 
    out << YAML::EndMap;
    return out;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const Constraint& c)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << c.name;
    out << YAML::Key << "function" << YAML::Value << c.getFunction(); 
    out << YAML::Key << "tool_feature" << YAML::Value << c.tool_feature; 
    out << YAML::Key << "object_feature" << YAML::Value << c.object_feature;
    out << YAML::EndMap;
    return out;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const Ranges& r)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "pos_low" << YAML::Value << r.pos_lo;
    out << YAML::Key << "pos_high" << YAML::Value << r.pos_hi; 
    out << YAML::Key << "weight" << YAML::Value << r.weight; 
    out << YAML::Key << "max_vel" << YAML::Value << r.max_vel;
    out << YAML::Key << "min_vel" << YAML::Value << r.min_vel;
    out << YAML::EndMap;
    return out;
  }

  void operator >> (const YAML::Node& node, KDL::Vector& v)
  {
    node[0] >> v[0];
    node[1] >> v[1];
    node[2] >> v[2];
  }

  void operator >> (const YAML::Node& node, KDL::Rotation& r)
  {
    double x,y,z,w;
    node[0] >> x;
    node[1] >> y;
    node[2] >> z;
    node[3] >> w;
    r = KDL::Rotation::Quaternion(x, y, z, w);
  }

  void operator >> (const YAML::Node& node, KDL::Frame& f)
  {
    node["translation"] >> f.p;
    node["orientation"] >> f.M;
  }

  void operator >> (const YAML::Node& node, KDL::JntArray& j)
  {
    j.resize(node.size());
    for(unsigned int i=0; i<node.size(); i++)
      node[i] >> j(i);
  }
  void operator >> (const YAML::Node& node, Feature& f)
  {
    node["name"] >> f.name;
    node["position"] >> f.pos;
    node["direction"] >> f.dir;
  }

  void operator >> (const YAML::Node& node, Constraint& c)
  {
    node["name"] >> c.name;
    std::string function_name;
    node["function"] >> function_name;
    c.setFunction(function_name);
    node["tool_feature"] >> c.tool_feature;
    node["object_feature"] >> c.object_feature;
  }

  void operator >> (const YAML::Node& node, Ranges& r)
  {
    node["pos_low"] >> r.pos_lo;
    node["pos_high"] >> r.pos_hi;
    node["weight"] >> r.weight;
    node["max_vel"] >> r.max_vel;
    node["min_vel"] >> r.min_vel;
  }

} /* namespace openrave_ompl_bridge */

#endif // OPENRAVE_OMPL_BRIDGE_YAML_UTILS_H
