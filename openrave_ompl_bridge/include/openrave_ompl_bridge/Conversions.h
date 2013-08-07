#ifndef OPENRAVE_OMPL_BRIDGE_CONVERSIONS_H
#define OPENRAVE_OMPL_BRIDGE_CONVERSIONS_H

#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>

#include <vector>
#include <openrave-core.h>

namespace openrave_ompl_bridge
{
  void toKDL(KDL::Jacobian& jacobian, const std::vector<double>& position_jacobian, const std::vector<double>& angular_jacobian);
  
  void toKDL(KDL::Frame& frame, const OpenRAVE::Transform& transform);
  KDL::Frame toKDL(const OpenRAVE::Transform& transform);
  void toOR(OpenRAVE::Transform& transform, const KDL::Frame& frame);
  OpenRAVE::Transform toOR(const KDL::Frame& frame);
} // namespace openrave_ompl_bridge
#endif // OPENRAVE_OMPL_BRIDGE_CONVERSIONS_H
