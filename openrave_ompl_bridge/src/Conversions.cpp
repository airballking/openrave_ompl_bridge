#include <openrave_ompl_bridge/Conversions.h>

namespace openrave_ompl_bridge
{
 void toKDL(KDL::Jacobian& jacobian, const std::vector<double>& position_jacobian, const std::vector<double>& angular_jacobian)
 {
   assert(jacobian.columns()*3 == position_jacobian.size()); 
   assert(jacobian.columns()*3 == angular_jacobian.size()); 
 
   for (unsigned int i=0; i<jacobian.columns(); i++)
   {
     jacobian(0, i) = position_jacobian[i];
     jacobian(1, i) = position_jacobian[i+jacobian.columns()];
     jacobian(2, i) = position_jacobian[i+2*jacobian.columns()];
     jacobian(3, i) = angular_jacobian[i];
     jacobian(4, i) = angular_jacobian[i+jacobian.columns()];
     jacobian(5, i) = angular_jacobian[i+2*jacobian.columns()];
   }
 }
 
 void toKDL(KDL::Frame& frame, const OpenRAVE::Transform& transform)
 {
   frame.p.x(transform.trans.x);
   frame.p.y(transform.trans.y);
   frame.p.z(transform.trans.z);
 
   frame.M = KDL::Rotation::Quaternion(transform.rot.w, transform.rot.y, transform.rot.z, transform.rot.x);
 }
 
 KDL::Frame toKDL(const OpenRAVE::Transform& transform)
 {
   KDL::Frame result;
   toKDL(result, transform);
   return result;
 }
 
 void toOR(OpenRAVE::Transform& transform, const KDL::Frame& frame)
 {
   transform.trans.x = frame.p.x();
   transform.trans.y = frame.p.y();
   transform.trans.z = frame.p.z();
 
   double x,y,z,w;
   frame.M.GetQuaternion(x, y, z, w);
   transform.rot.x = w;
   transform.rot.y = y;
   transform.rot.z = z;
   transform.rot.w = x;
 }
 
 OpenRAVE::Transform toOR(const KDL::Frame& frame)
 {
   OpenRAVE::Transform result;
   toOR(result, frame);
   return result;
 }
} // namespace openrave_ompl_bridge
