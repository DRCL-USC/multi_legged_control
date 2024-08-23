
#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2
{
    namespace planner
    {

        class Obstacles : public SolverSynchronizedModule
        {
        public:
            Obstacles(std::vector<std::pair<scalar_t, scalar_t>> obstacles_pose,
                      bool is_static = true) : obstacles_pose_init(obstacles_pose), obstacles_pose_(obstacles_pose), is_static_(is_static) {};

            void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                              const ReferenceManagerInterface &referenceManager) override
            {
                if (!is_static_)
                {
                    for (int i = 0; i < obstacles_pose_init.size(); i++)
                    {
                        obstacles_pose_[i].second = obstacles_pose_init[i].second + 2 * sin(initTime * 2 * M_PI / 10);
                        //    std::cout << "Obstacle " << i << ": "<< obstacles_pose_[i].first << " " << obstacles_pose_[i].second << std::endl;
                    }
                }
            }

            void postSolverRun(const PrimalSolution &primalSolution) override {
            };

            std::vector<std::pair<scalar_t, scalar_t>> getObstacles() const
            {
                return obstacles_pose_;
            }

        private:
            std::vector<std::pair<scalar_t, scalar_t>> obstacles_pose_init;
            std::vector<std::pair<scalar_t, scalar_t>> obstacles_pose_;
            bool is_static_;
        };

    } // namespace planner
} // namespace ocs2
