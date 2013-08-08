#include <openrave_ompl_bridge/Environment.h>
#include <openrave_ompl_bridge/Conversions.h>

namespace openrave_ompl_bridge
{
  Environment::Environment(OpenRAVE::EnvironmentBasePtr environment)
  {
    env_ = environment;
  }

  Environment::~Environment()
  {
  }

  KDL::Frame Environment::getObjectFrameInWorld(const std::string& object_name)
  {
    OpenRAVE::KinBodyPtr body = env_->GetKinBody(object_name);
    assert(body);
    return toKDL(body->GetTransform());
  }

  KDL::Frame Environment::getTransform(const std::string& parent_frame, const std::string& child_frame)
  {
    KDL::Frame parent = getObjectFrameInWorld(parent_frame);
    KDL::Frame child = getObjectFrameInWorld(child_frame);
    return parent.Inverse()*child;
  }
} // namespace openrave_ompl_bridge
