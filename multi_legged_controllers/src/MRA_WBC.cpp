#include "multi_legged_controllers/MRA_WBC.h"

namespace legged {

MRA_WBC::MRA_WBC(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics &eeKinematics)
    : WbcBase(pinocchioInterface, info, eeKinematics) {
    alpha_hat.setZero();
    beta_hat.setZero();
    delta_hat.setZero();

    H.resize(12, 6);
    H.setZero();
    H.block(6, 0, 6, 6) = Matrix6::Identity();
    H.block(0, 0, 6, 6) = Matrix6::Identity() * 3;

    K_alpha.resize(6, 12);
    K_alpha.setZero();
    K_alpha.block(0, 0, 6, 6) = Matrix6::Identity() * 3;
    K_alpha.block(0, 6, 6, 6) = Matrix6::Identity();
}

void MRA_WBC::loadTasksSetting(const std::string &taskFile, bool verbose) {
    WbcBase::loadTasksSetting(taskFile, verbose);

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "weight.";
    if (verbose) {
        std::cerr << "\n #### WBC weight:";
        std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
    loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
    loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);

    loadData::loadPtreeValue(pt, P_adaptive, "adaptive.P", verbose);
    loadData::loadPtreeValue(pt, epsilon_theta, "adaptive.epsilon_theta", verbose);
    loadData::loadPtreeValue(pt, theta_max, "adaptive.theta_max", verbose);
    loadData::loadPtreeValue(pt, lambda, "adaptive.lambda", verbose);

    loadData::loadEigenMatrix(taskFile, "adaptive.gamma", gamma);
}

Task MRA_WBC::formulateBaseAccelMRATask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period) {
    matrix_t a(6, numDecisionVars_);
    a.setZero();
    a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

    vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
    inputLast_ = inputDesired;
    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

    const auto &model = pinocchioInterfaceDesired_.getModel();
    auto &data = pinocchioInterfaceDesired_.getData();
    const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
    const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    const auto &A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
    const Matrix6 Ab = A.template leftCols<6>();
    const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
    const auto Aj = A.rightCols(info_.actuatedDofNum);
    const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
    Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    centroidalMomentumRate.noalias() -= ADot * vDesired;
    centroidalMomentumRate.noalias() -= Aj * jointAccel;

    vector_t eta(12);
    eta << qDesired.head(6) - qMeasured_.head(6), vDesired.head(6) - vMeasured_.head(6);

    Q_alpha = (K_alpha * eta.cwiseAbs()).asDiagonal();

    vector_t y_alpha = Q_alpha * H.transpose() * P_adaptive * eta;
    vector_t y_beta = H.transpose() * P_adaptive * eta;

    alpha_hat = projection(alpha_hat, y_alpha, period);
    beta_hat = projection(beta_hat, y_beta, period);

    delta_hat = Q_alpha * alpha_hat + beta_hat;

    Vector6 b = AbInv * centroidalMomentumRate + delta_hat;

    return {a, b, matrix_t(), vector_t()};
}

vector_t MRA_WBC::projection(Vector6 &theta, vector_t &y, scalar_t period) {
    scalar_t f_theta = (theta.norm() * theta.norm() - theta_max * theta_max) / (epsilon_theta * theta_max * theta_max);
    vector_t df_theta = 2 * theta / (epsilon_theta * theta_max * theta_max);

    vector_t theta_dot;
    if (f_theta > 0 && y.dot(df_theta) > 0) {
        theta_dot = gamma.asDiagonal() * (y - (f_theta / df_theta.dot(df_theta)) * (df_theta * df_theta.transpose()) * y);
    } else {
        theta_dot = gamma.asDiagonal() * y;
    }

    theta += theta_dot * period;

    return theta;
}

Task MRA_WBC::formulateMRAWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period) {
    return formulateSwingLegTask() * weightSwingLeg_ +
           formulateBaseAccelMRATask(stateDesired, inputDesired, period) * weightBaseAccel_ +
           formulateContactForceTask(inputDesired) * weightContactForce_;
}

Task MRA_WBC::CLFConstraint(const vector_t &stateDesired, const vector_t &inputDesired) {
    const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
    const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    vector_t eta(12);
    eta << qDesired.head(6) - qMeasured_.head(6), vDesired.head(6) - vMeasured_.head(6);

    scalar_t lyp = eta.transpose() * P_adaptive * eta;

    matrix_t D(12, 12);
    D.setZero();
    D.block(0, 6, 6, 6) = matrix_t::Identity(6, 6);

    matrix_t G(12, 6);
    G.setZero();
    G.block(6, 0, 6, 6) = matrix_t::Identity(6, 6);

    scalar_t lyp_dot_1 = eta.transpose() * P_adaptive * (D + D.transpose()) * eta;

    matrix_t d(1, numDecisionVars_);
    d.setZero();
    d.block(0, 0, 1, 6) = 2 * eta.transpose() * P_adaptive * G;

    vector_t f(1);
    f << -lyp_dot_1 - lambda * lyp;

    return {matrix_t(), vector_t(), d, f};
}

Task MRA_WBC::formulateMRAConstraints(const vector_t &stateDesired, const vector_t &inputDesired) {
    return formulateFloatingBaseEomTask() +
           formulateTorqueLimitsTask() +
           formulateFrictionConeTask() +
           formulateNoContactMotionTask();
}

vector_t MRA_WBC::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode, scalar_t period) {
    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

    Task constraints = formulateMRAConstraints(stateDesired, inputDesired);
    size_t numConstraints = constraints.b_.size() + constraints.f_.size();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
    vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
    A << constraints.a_,
         constraints.d_;

    lbA << constraints.b_,
          -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
    ubA << constraints.b_,
          constraints.f_; // clang-format on

    Task weighedTask = formulateMRAWeightedTasks(stateDesired, inputDesired, period);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
    vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

    auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    options.enableEqualities = qpOASES::BT_TRUE;
    qpProblem.setOptions(options);
    int nWsr = 20;

    qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
    vector_t qpSol(getNumDecisionVars());

    qpProblem.getPrimalSolution(qpSol.data());
    return qpSol;
}

} // namespace legged
