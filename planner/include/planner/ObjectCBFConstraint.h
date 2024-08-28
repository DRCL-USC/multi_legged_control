
#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <planner/Obstacles.h>

namespace ocs2
{
  namespace planner
  {

    class ObjectCBFConstraint final : public StateConstraint
    {
    public:
      ObjectCBFConstraint(std::shared_ptr<Obstacles> obstacles, scalar_array_t radius_array)
          : StateConstraint(ConstraintOrder::Quadratic), radius_array_(radius_array), obstacles_(obstacles) {};

      ~ObjectCBFConstraint() override = default;
      ObjectCBFConstraint *clone() const override { return new ObjectCBFConstraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return obstacles_->getObstacles().size(); };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        auto pos_array_ = obstacles_->getObstacles();

        vector_t constraint(pos_array_.size());

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          scalar_t B = -(radius_array_[i] + 0.05) * (radius_array_[i] + 0.05) +
                       (state(0) - pos_array_[i](0)) * (state(0) - pos_array_[i](0)) +
                       (state(1) - pos_array_[i](1)) * (state(1) - pos_array_[i](1)) +
                       (state(2) - pos_array_[i](2)) * (state(2) - pos_array_[i](2));
          constraint(i) = B +
                          2 * (state(0) - pos_array_[i](0)) * state(7) +
                          2 * (state(1) - pos_array_[i](1)) * state(8) +
                          2 * (state(2) - pos_array_[i](2)) * state(9);
        }

        return constraint;
      };

      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;

        auto pos_array_ = obstacles_->getObstacles();
        linearApproximation.f = getValue(time, state, preComp);

        matrix_t C(pos_array_.size(), state.size());

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          C.row(i) << 2 * (state(0) - pos_array_[i](0)) + 2 * state(7),
              2 * (state(1) - pos_array_[i](1)) + 2 * state(8),
              2 * (state(2) - pos_array_[i](2)) + 2 * state(9),
              0, 0, 0, 0,
              2 * (state(0) - pos_array_[i](0)),
              2 * (state(1) - pos_array_[i](1)),
              2 * (state(2) - pos_array_[i](2)),
              0, 0, 0;
        }

        linearApproximation.dfdx = C;
        return linearApproximation;
      };

      VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t &state,
                                                                     const PreComputation &preComp) const override
      {
        VectorFunctionQuadraticApproximation quadraticApproximation;
        auto pos_array_ = obstacles_->getObstacles();

        quadraticApproximation.f = getValue(time, state, preComp);

        quadraticApproximation.dfdx = getLinearApproximation(time, state, preComp).dfdx;

        matrix_t dC = matrix_t::Zero(state.size(), state.size());
        dC(0, 0) = 2;
        dC(1, 1) = 2;
        dC(2, 2) = 2;
        
        dC(0, 7) = 2;
        dC(1, 8) = 2;
        dC(2, 9) = 2;

        dC(7, 0) = 2;
        dC(8, 1) = 2;
        dC(9, 2) = 2;

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          quadraticApproximation.dfdxx.emplace_back(dC);
        }

        return quadraticApproximation;
      }

    private:
      ObjectCBFConstraint(const ObjectCBFConstraint &other) = default;
      const scalar_array_t radius_array_;
      scalar_t alpha_;
      std::shared_ptr<Obstacles> obstacles_;
    };

  } // namespace planner
} // namespace ocs2
