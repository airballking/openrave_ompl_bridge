#ifndef _OPENRAVE_OMPL_BRIDGE_CBIRRT_SPACE_
#define _OPENRAVE_OMPL_BRIDGE_CBIRRT_SPACE_
 
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

namespace openrave_ompl_bridge
{
  // convenience typedef for task-function-based projection of configurations
  typedef boost::function< void (ompl::base::State *)> ConstraintProjectionFn;
  // convenience typedef for checking whether a state fulfills the path constraints
  typedef boost::function< bool (const ompl::base::State *)> PathConstraintsCheckFn;

  class CBiRRTSpace : public ompl::base::CompoundStateSpace
  {
    public:
      // The StateType-class required for all ompl-StateSpaces used to access
      // state-objects of this StateSpace.
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

      // tons of convenience functions to set and get the basics of this space     
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

      // all the ompl::base::State-function we need to overwrite
      virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const; 
      virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const;
      virtual ompl::base::State* allocState(void) const;
      virtual void freeState(ompl::base::State *state) const;

      // constraint-related function-hook
      void setConstraintProjectionFunction (const ConstraintProjectionFn &projection_function);
      void setPathConstraintsCheckFunction (const PathConstraintsCheckFn &path_constraints_check_function); 
      // usage of constraint-related function hook
      void constraintProjectConfiguration(ompl::base::State *state) const;
      bool fulfillsPathConstraints(const ompl::base::State *state) const;

    private:
      ConstraintProjectionFn projection_function_;
      PathConstraintsCheckFn path_constraints_check_function_;
  };

  // convenience typedef for shared pointers to objects of this class
  typedef boost::shared_ptr<CBiRRTSpace> CBiRRTSpacePtr;

} // namespace openrave_ompl_bridge

#endif // _OPENRAVE_OMPL_BRIDGE_CBIRRT_SPACE_
