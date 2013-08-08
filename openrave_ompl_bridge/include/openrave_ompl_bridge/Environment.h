#ifndef OPENRAVE_OMPL_BRIDGE_ENVIRONMENT_H
#define OPENRAVE_OMPL_BRIDGE_ENVIRONMENT_H

#include <openrave-core.h>
#include <kdl/frames.hpp>
#include <string>

namespace openrave_ompl_bridge
{
  class Environment
  {
    public:
      Environment(OpenRAVE::EnvironmentBasePtr environment);
      ~Environment();

      KDL::Frame getObjectFrameInWorld(const std::string& object_name);
      KDL::Frame getTransform(const std::string& parent_frame, const std::string& child_frame);

    private:
      OpenRAVE::EnvironmentBasePtr env_;
  };

  typedef boost::shared_ptr<Environment> EnvironmentPtr;
  typedef boost::shared_ptr<const Environment> ConstEnvironmentPtr;

} /* namespace openrave_ompl_bridge */

#endif // OPENRAVE_OMPL_BRIDGE_ENVIRONMENT_H
