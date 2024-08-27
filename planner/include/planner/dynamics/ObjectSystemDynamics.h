#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "planner/definitions.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ocs2
{
  namespace planner
  {
    class ObjectSytemDynamics : public SystemDynamicsBaseAD
    {
    public:
      ObjectSytemDynamics(const std::string &libraryFolder, bool verbose)
      {
        initialize(STATE_DIM, INPUT_DIM, "object_dynamics", libraryFolder, true, verbose);
      }

      ~ObjectSytemDynamics() override = default;

      ObjectSytemDynamics(const ObjectSytemDynamics &rhs) = default;

      ObjectSytemDynamics *clone() const override { return new ObjectSytemDynamics(*this); }

      ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input,
                                const ad_vector_t &parameters) const override
      {
        // dxdt
        ad_vector_t stateDerivative(STATE_DIM);
        Eigen::Quaternion<ad_scalar_t> quat(state(3), state(4), state(5), state(6));
        Eigen::Quaternion<ad_scalar_t> omegaQuat(static_cast<ad_scalar_t>(0.0), state(10), state(11), state(12));
        Eigen::Quaternion<ad_scalar_t> quat_dot = (quat * omegaQuat);
        stateDerivative << state.segment<3>(7),
            static_cast<ad_scalar_t>(0.5) * quat_dot.w(), static_cast<ad_scalar_t>(0.5) * quat_dot.x(), static_cast<ad_scalar_t>(0.5) * quat_dot.y(), static_cast<ad_scalar_t>(0.5) * quat_dot.z(),
            input;
        return stateDerivative;
      }
    };

  } // namespace planner
} // namespace ocs2
