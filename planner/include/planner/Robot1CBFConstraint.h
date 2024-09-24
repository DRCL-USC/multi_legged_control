
#pragma once

#include <planner/ObjectCBFConstraint.h>

namespace ocs2
{
  namespace planner
  {

    class Robot1CBFConstraint final : public StateConstraint
    {
    public:
      Robot1CBFConstraint(std::shared_ptr<Obstacles> obstacles, scalar_array_t radius_array)
          : StateConstraint(ConstraintOrder::Quadratic), radius_array_(radius_array), obstacles_(obstacles) {};

      ~Robot1CBFConstraint() override = default;
      Robot1CBFConstraint *clone() const override { return new Robot1CBFConstraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return obstacles_->getObstacles().size(); };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        auto pos_array_ = obstacles_->getObstacles();

        vector_t constraint(pos_array_.size());

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          scalar_t B = -(radius_array_[i] + 0.2) * (radius_array_[i] + 0.2) +
                       (state(0) - pos_array_[i](0)) * (state(0) - pos_array_[i](0)) +
                       (state(1) - pos_array_[i](1)) * (state(1) - 0.5 - pos_array_[i](1));
          constraint(i) = B +
                          2 * (state(0) - pos_array_[i](0)) * state(7) +
                          2 * (state(1) - 0.5 - pos_array_[i](1)) * state(8);
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
              2 * (state(1) - 0.5 - pos_array_[i](1)) + 2 * state(8),
              0,
              0, 0, 0, 0,
              2 * (state(0) - pos_array_[i](0)),
              2 * (state(1) - 0.5 - pos_array_[i](1)),
              0,
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
        
        dC(0, 7) = 2;
        dC(1, 8) = 2;

        dC(7, 0) = 2;
        dC(8, 1) = 2;

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          quadraticApproximation.dfdxx.emplace_back(dC);
        }

        return quadraticApproximation;
      }

    private:
      Robot1CBFConstraint(const Robot1CBFConstraint &other) = default;
      const scalar_array_t radius_array_;
      scalar_t alpha_;
      std::shared_ptr<Obstacles> obstacles_;
    };

  } // namespace planner
} // namespace ocs2
