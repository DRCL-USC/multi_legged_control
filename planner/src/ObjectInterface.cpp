#include <iostream>
#include <string>

#include "planner/ObjectInterface.h"

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <planner/dynamics/ObjectSystemDynamics.h>
#include <planner/ObjectBoundConstraint.h>
#include <planner/ObjectCBFConstraint.h>
#include <planner/Robot1CBFConstraint.h>
#include <planner/Robot2CBFConstraint.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2
{
  namespace planner
  {

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ObjectInterface::ObjectInterface(const std::string &taskFile, const std::string &libraryFolder, bool verbose)
    {
      // check that task file exists
      boost::filesystem::path taskFilePath(taskFile);
      if (boost::filesystem::exists(taskFilePath))
      {
        std::cerr << "[ObjectInterface] Loading task file: " << taskFilePath << "\n";
      }
      else
      {
        throw std::invalid_argument("[ObjectInterface] Task file not found: " + taskFilePath.string());
      }
      // create library folder if it does not exist
      boost::filesystem::path libraryFolderPath(libraryFolder);
      boost::filesystem::create_directories(libraryFolderPath);
      std::cerr << "[ObjectInterface] Generated library path: " << libraryFolderPath << "\n";

      // DDP-MPC settings
      ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
      mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);

      /*
       * ReferenceManager & SolverSynchronizedModule
       */
      referenceManagerPtr_.reset(new ReferenceManager);

      /*
       * Optimal control problem
       */
      // Cost
      matrix_t Q(STATE_DIM, STATE_DIM);
      matrix_t R(INPUT_DIM, INPUT_DIM);
      matrix_t Qf(STATE_DIM, STATE_DIM);
      loadData::loadEigenMatrix(taskFile, "Q", Q);
      loadData::loadEigenMatrix(taskFile, "R", R);
      loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
      if (verbose)
      {
        std::cerr << "Q:  \n"
                  << Q << "\n";
        std::cerr << "R:  \n"
                  << R << "\n";
        std::cerr << "Q_final:\n"
                  << Qf << "\n";
      }

      problem_.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
      problem_.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(Qf));

      // Dynamics
      problem_.dynamicsPtr.reset(new ObjectSytemDynamics(libraryFolder, verbose));

      // Rollout
      auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", verbose);
      rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

      // Constraints

      // Bound constraints
      SquaredHingePenalty::Config boundsConfig;
      loadData::loadCppDataType(taskFile, "penalty_config.bounds.mu", boundsConfig.mu);
      loadData::loadCppDataType(taskFile, "penalty_config.bounds.delta", boundsConfig.delta);
      problem_.stateSoftConstraintPtr->add("state_bound",
                                           std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<ObjectBoundConstraint>(taskFile),
                                                                                              std::make_unique<SquaredHingePenalty>(boundsConfig))));

      // CBFs
      SquaredHingePenalty::Config cbfConfig;
      loadData::loadCppDataType(taskFile, "penalty_config.cbf.mu", cbfConfig.mu);
      loadData::loadCppDataType(taskFile, "penalty_config.cbf.delta", cbfConfig.delta);

      vector_array_t obstacles_pose_array;
      matrix_t obstacles_pose_matrix(OBSTACLES_COUNT, 3);
      loadData::loadEigenMatrix(taskFile, "obstacles.pose", obstacles_pose_matrix);
      for (int i = 0; i < obstacles_pose_matrix.rows(); ++i)
      {
        obstacles_pose_array.push_back(obstacles_pose_matrix.row(i));
      }

      scalar_array_t obstacles_radius;
      loadData::loadStdVector(taskFile, "obstacles.radius", obstacles_radius, verbose);

      // Obstacles
      obstaclesPtr_.reset(new Obstacles(obstacles_pose_array));

      problem_.stateSoftConstraintPtr->add("Obstacle_cbf",
                                           std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<ObjectCBFConstraint>(obstaclesPtr_, obstacles_radius),
                                                                                              std::make_unique<SquaredHingePenalty>(cbfConfig))));

      // 2d obstacles
      SquaredHingePenalty::Config cbf2dConfig;
      loadData::loadCppDataType(taskFile, "2d_penalty_config.mu", cbf2dConfig.mu);
      loadData::loadCppDataType(taskFile, "2d_penalty_config.delta", cbf2dConfig.delta);

      vector_array_t obstacles2d_pose_array;
      matrix_t obstacles2d_pose_matrix(1, 3);
      loadData::loadEigenMatrix(taskFile, "obstacles_2d.pose", obstacles2d_pose_matrix);
      for (int i = 0; i < obstacles2d_pose_matrix.rows(); ++i)
      {
        obstacles2d_pose_array.push_back(obstacles2d_pose_matrix.row(i));
      }

      scalar_array_t obstacles2d_radius;
      loadData::loadStdVector(taskFile, "obstacles_2d.radius", obstacles2d_radius, verbose);

      // Obstacles
      obstacles2dPtr_.reset(new Obstacles(obstacles2d_pose_array));

      // Robot 1
      problem_.stateSoftConstraintPtr->add("Robot1_cbf",
                                           std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<Robot1CBFConstraint>(obstacles2dPtr_, obstacles2d_radius),
                                                                                              std::make_unique<SquaredHingePenalty>(cbf2dConfig))));

      // Robot 2
      problem_.stateSoftConstraintPtr->add("Robot2_cbf",
                                           std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<Robot2CBFConstraint>(obstacles2dPtr_, obstacles2d_radius),
                                                                                              std::make_unique<SquaredHingePenalty>(cbf2dConfig))));

      // Initialization
      objectInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
    }

  } // namespace planner
} // namespace ocs2
