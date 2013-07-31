#ifndef _OPENRAVE_OMPL_BRIDGE_CBIRRT_SPACE_
#define _OPENRAVE_OMPL_BRIDGE_CBIRRT_SPACE_
 
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

namespace openrave_ompl_bridge
{
  class CBiRRTSpace : public ompl::base::CompoundStateSpace
  {
    public:
      class StateType : public ompl::base::CompoundStateSpace::StateType
      {
        public:
        StateType(void) : ompl::base::CompoundStateSpace::StateType()
        {
        }
       
        double getJointValue(unsigned int index) const
        {
          return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[index];
        }
       
        double getConstraintValue(unsigned int index) const
        {
          return as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index];
        }
       
        void setJointValue(double joint_value, unsigned int index)
        {
          as<ompl::base::RealVectorStateSpace::StateType>(0)->values[index] = joint_value;
        }
       
        void setConstraintValue(double constraint_value, unsigned int index)
        {
          as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index] = constraint_value;
        }
      };

      CBiRRTSpace(unsigned int number_of_joints, unsigned int number_of_constraints) : ompl::base::CompoundStateSpace()
      {
        setName("CBiRRT" + getName());
        // creating a new state type by choosing a number bigger than COUNT
        type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;
        // setting weights of sub-spaces such that distance is solely joint space distance
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(number_of_joints)), 1.0);
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(number_of_constraints)), 0.0);
        // make sure nobody changes the structure of the space
        lock();
      }
     
      virtual ~CBiRRTSpace(void)
      {
      }
     
      void setBounds(const ompl::base::RealVectorBounds &joint_bounds, const ompl::base::RealVectorBounds &constraint_bounds)
      {
        as<ompl::base::RealVectorStateSpace>(0)->setBounds(joint_bounds);
        as<ompl::base::RealVectorStateSpace>(1)->setBounds(constraint_bounds);
      }
     
      const ompl::base::RealVectorBounds& getJointBounds(void) const
      {
        return as<ompl::base::RealVectorStateSpace>(0)->getBounds();
      }

      const ompl::base::RealVectorBounds& getConstraintBounds(void) const
      {
        return as<ompl::base::RealVectorStateSpace>(1)->getBounds();
      }

      unsigned int getNumberOfJoints(void) const
      {
        return as<ompl::base::RealVectorStateSpace>(0)->getDimension();
      }

      unsigned int getNumberOfConstraints(void) const
      {
        return as<ompl::base::RealVectorStateSpace>(1)->getDimension();
      }

      virtual ompl::base::State* allocState(void) const;
      virtual void freeState(ompl::base::State *state) const;
  };

  // convenience typedef for shared pointers to objects of this class
  typedef boost::shared_ptr<CBiRRTSpace> CBiRRTSpacePtr;

} // namespace openrave_ompl_bridge

#endif // _OPENRAVE_OMPL_BRIDGE_CBIRRT_SPACE_

