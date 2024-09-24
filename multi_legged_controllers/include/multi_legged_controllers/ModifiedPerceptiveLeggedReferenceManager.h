
#include <legged_perceptive_interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace legged
{

  using namespace ocs2;
  using namespace legged_robot;

  class ModifiedPerceptiveLeggedReferenceManager : public PerceptiveLeggedReferenceManager
  {
  public:
    ModifiedPerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                             std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                             const EndEffectorKinematics<scalar_t> &endEffectorKinematics, scalar_t comHeight)
        : PerceptiveLeggedReferenceManager(info, gaitSchedulePtr, swingTrajectoryPtr, convexRegionSelectorPtr, endEffectorKinematics, comHeight) {}

  protected:
    void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t &initState, TargetTrajectories &targetTrajectories, ModeSchedule &modeSchedule) override
    {
      const auto timeHorizon = finalTime - initTime;
      modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

      TargetTrajectories newTargetTrajectories;
      int nodeNum = 11;
      for (size_t i = 0; i < nodeNum; ++i)
      {
        scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
        vector_t state = targetTrajectories.getDesiredState(time);
        vector_t input = targetTrajectories.getDesiredState(time);

        const auto &map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
        vector_t pos = centroidal_model::getBasePose(state, info_).head(3);

        // Base Orientation
        scalar_t step = 0.3;
        grid_map::Vector3 normalVector;
        normalVector(0) = (map.atPosition("smooth_planar", pos + grid_map::Position(-step, 0)) -
                           map.atPosition("smooth_planar", pos + grid_map::Position(step, 0))) /
                          (2 * step);
        normalVector(1) = (map.atPosition("smooth_planar", pos + grid_map::Position(0, -step)) -
                           map.atPosition("smooth_planar", pos + grid_map::Position(0, step))) /
                          (2 * step);
        normalVector(2) = 1;
        normalVector.normalize();
        matrix3_t R;
        scalar_t z = centroidal_model::getBasePose(state, info_)(3);
        R << cos(z), -sin(z), 0, // clang-format off
             sin(z), cos(z), 0,
             0, 0, 1; // clang-format on
        vector_t v = R.transpose() * normalVector;
        centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());

        // Base Z Position
        // centroidal_model::getBasePose(state, info_)(2) =
        //     map.atPosition("smooth_planar", pos) + comHeight_ / cos(centroidal_model::getBasePose(state, info_)(4));

        newTargetTrajectories.timeTrajectory.push_back(time);
        newTargetTrajectories.stateTrajectory.push_back(state);
        newTargetTrajectories.inputTrajectory.push_back(input);
      }
      targetTrajectories = newTargetTrajectories;

      // Footstep
      convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

      // Swing trajectory
      updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
    };
  };
} // namespace legged
