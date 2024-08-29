#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// planenr
#include "planner/definitions.h"
#include "planner/ObjectCBFConstraint.h"

namespace ocs2 {
namespace planner {

class ObjectInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] verbose: Flag to determine to print out the loaded settings and status of complied libraries.
   */
  ObjectInterface(const std::string& taskFile, const std::string& libraryFolder, bool verbose);

  /**
   * Destructor
   */
  ~ObjectInterface() override = default;

  ddp::Settings& ddpSettings() { return ddpSettings_; }

  mpc::Settings& mpcSettings() { return mpcSettings_; }

  OptimalControlProblem& optimalControlProblem() { return problem_; }
  const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const RolloutBase& getRollout() const { return *rolloutPtr_; }

  const Initializer& getInitializer() const override { return *objectInitializerPtr_; }

  std::shared_ptr<Obstacles> getObstaclesPtr() { return obstaclesPtr_; }

 private:
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;

  OptimalControlProblem problem_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> objectInitializerPtr_;
  std::shared_ptr<Obstacles> obstaclesPtr_;
};

}  // namespace planner
}  // namespace ocs2
