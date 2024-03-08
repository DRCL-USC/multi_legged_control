
#include <legged_interface/LeggedInterface.h>
#include <legged_perceptive_interface/PerceptiveLeggedInterface.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <legged_interface/constraint/LeggedSelfCollisionConstraint.h>

namespace legged
{
    using namespace ocs2;
    using namespace legged_robot;

    std::unique_ptr<StateCost> getSelfCollisionConstraintCorrected(const std::string ns, CentroidalModelInfo centroidalModelInfo_, const PinocchioInterface &pinocchioInterface,
                                                                   const std::string &taskFile, const std::string &prefix,
                                                                   bool verbose)
    {
        std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
        std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
        scalar_t mu = 1e-2;
        scalar_t delta = 1e-3;
        scalar_t minimumDistance = 0.0;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        if (verbose)
        {
            std::cerr << "\n #### SelfCollision Settings: ";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, mu, prefix + ".mu", verbose);
        loadData::loadPtreeValue(pt, delta, prefix + ".delta", verbose);
        loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", verbose);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, verbose);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, verbose);

        for (auto &pair : collisionLinkPairs)
        {
            pair.first = ns + "_" + pair.first;
            pair.second = ns + "_" + pair.second;
        }
        std::cout << "collisionLinkPairs: " << collisionLinkPairs[0].first << std::endl;

        std::unique_ptr<PinocchioGeometryInterface> geometryInterfacePtr_ = std::make_unique<PinocchioGeometryInterface>(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
            const size_t numCollisionPairs = geometryInterfacePtr_->getNumCollisionPairs();
            std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";
        }

        std::unique_ptr<StateConstraint> constraint = std::make_unique<LeggedSelfCollisionConstraint>(
            CentroidalModelPinocchioMapping(centroidalModelInfo_), *geometryInterfacePtr_, minimumDistance);

        auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
    }
    class MultiLeggedInterface : public LeggedInterface
    {
    public:
        MultiLeggedInterface(const std::string ns, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                             bool useHardFrictionConeConstraint = false)
            : LeggedInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint),
              ns_(ns)
        {
            modelSettings_.contactNames3DoF = {ns + "_LF_FOOT", ns + "_RF_FOOT", ns + "_LH_FOOT", ns + "_RH_FOOT"};
            modelSettings_.jointNames = {ns + "_LF_HAA", ns + "_LF_HFE", ns + "_LF_KFE", ns + "_RF_HAA", ns + "_RF_HFE", ns + "_RF_KFE",
                                         ns + "_LH_HAA", ns + "_LH_HFE", ns + "_LH_KFE", ns + "_RH_HAA", ns + "_RH_HFE", ns + "_RH_KFE"};
        }

        void setupOptimalControlProblem(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose) override
        {
            LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
            problemPtr_->stateSoftConstraintPtr->add("selfCollisionCorrected",
                                                     getSelfCollisionConstraintCorrected(ns_, centroidalModelInfo_, *pinocchioInterfacePtr_, taskFile, "selfCollision", verbose));
        }

    private:
        const std::string ns_;
    };

    class MultiPerceptiveLeggedInterface : public PerceptiveLeggedInterface
    {
    public:
        MultiPerceptiveLeggedInterface(const std::string ns, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                       bool useHardFrictionConeConstraint = false)
            : PerceptiveLeggedInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint),
              ns_(ns)
        {
            modelSettings_.contactNames3DoF = {ns + "_LF_FOOT", ns + "_RF_FOOT", ns + "_LH_FOOT", ns + "_RH_FOOT"};
            modelSettings_.jointNames = {ns + "_LF_HAA", ns + "_LF_HFE", ns + "_LF_KFE", ns + "_RF_HAA", ns + "_RF_HFE", ns + "_RF_KFE",
                                         ns + "_LH_HAA", ns + "_LH_HFE", ns + "_LH_KFE", ns + "_RH_HAA", ns + "_RH_HFE", ns + "_RH_KFE"};
        }

        void setupOptimalControlProblem(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose) override
        {
            PerceptiveLeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
            problemPtr_->stateSoftConstraintPtr->add("selfCollisionCorrected",
                                                     getSelfCollisionConstraintCorrected(ns_, centroidalModelInfo_, *pinocchioInterfacePtr_, taskFile, "selfCollision", verbose));
        }

    private:
        const std::string ns_;
    };

} // namespace legged
