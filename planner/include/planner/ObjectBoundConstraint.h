
#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2
{
  namespace planner
  {

    class ObjectBoundConstraint final : public StateConstraint
    {
    public:
      ObjectBoundConstraint(const std::string &taskFile)
          : StateConstraint(ConstraintOrder::Linear)
      {
        loadData::loadCppDataType(taskFile, "height_bounds.h_min", h_min);
        loadData::loadCppDataType(taskFile, "height_bounds.h_max", h_max);
      };

      ~ObjectBoundConstraint() override = default;
      ObjectBoundConstraint *clone() const override { return new ObjectBoundConstraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return 4; };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        const Eigen::Quaternion<scalar_t> quat = Eigen::Quaternion<scalar_t>(state(3), state(4), state(5), state(6));
        Eigen::Matrix<scalar_t, 3, 3> rotmat = quat.toRotationMatrix();

        scalar_t robot_COM_height[2];

        Eigen::Matrix<scalar_t, 3, 1> rod_tip_1 = rotmat * (Eigen::Matrix<scalar_t, 3, 1>() << 0.0, -0.5, 0.0).finished();
        Eigen::Matrix<scalar_t, 3, 1> rod_tip_2 = rotmat * (Eigen::Matrix<scalar_t, 3, 1>() << 0.0, 0.5, 0.0).finished();

        robot_COM_height[0] = state(2) + rod_tip_1(2) - 0.1;
        robot_COM_height[1] = state(2) + rod_tip_2(2) - 0.1;

        vector_t constraint(4);
        constraint << -h_min + robot_COM_height[0], h_max - robot_COM_height[0], -h_min + robot_COM_height[1], h_max - robot_COM_height[1];

        return constraint;
      };

      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;

        linearApproximation.f = getValue(time, state, preComp);

        matrix_t C(4, state.size());

        C.row(0) << 0, 0, 1, 0, 0, 0, 0, vector_t::Zero(6);
        C.row(1) << 0, 0, -1, 0, 0, 0, 0, vector_t::Zero(6);
        C.row(2) << 0, 0, 1, 0, 0, 0, 0, vector_t::Zero(6);
        C.row(3) << 0, 0, -1, 0, 0, 0, 0, vector_t::Zero(6);

        linearApproximation.dfdx = C;
        return linearApproximation;
      };

      // VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t &state,
      //                                                                const PreComputation &preComp) const override
      // {
      //   VectorFunctionQuadraticApproximation quadraticApproximation;
      //   auto pos_array_ = obstacles_->getObstacles();

      //   quadraticApproximation.f = getValue(time, state, preComp);

      //   quadraticApproximation.dfdx = getLinearApproximation(time, state, preComp).dfdx;

      //   matrix_t dC = matrix_t::Zero(state.size(), state.size());
      //   dC(0, 0) = 2;
      //   dC(3, 0) = 2;
      //   dC(1, 1) = 2;
      //   dC(4, 1) = 2;
      //   dC(0, 3) = 2;
      //   dC(1, 4) = 2;

      //   for (size_t i = 0; i < pos_array_.size(); ++i)
      //   {
      //     quadraticApproximation.dfdxx.emplace_back(dC);
      //   }

      //   return quadraticApproximation;
      // }

    private:
      ObjectBoundConstraint(const ObjectBoundConstraint &other) = default;
      scalar_t h_min, h_max;
    };

  } // namespace planner
} // namespace ocs2
