#ifndef LEG_WBC_MRA_WBC_H
#define LEG_WBC_MRA_WBC_H

#include <pinocchio/fwd.hpp>
#include <legged_wbc/WbcBase.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>
#include <qpOASES.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/exceptions.hpp>

namespace legged {

class MRA_WBC : public WbcBase {
public:
    using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
    using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

    MRA_WBC(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics &eeKinematics);

    void loadTasksSetting(const std::string &taskFile, bool verbose);

    Task formulateBaseAccelMRATask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);

    vector_t projection(Vector6 &theta, vector_t &y, scalar_t period);

    Task formulateMRAWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);

    Task CLFConstraint(const vector_t &stateDesired, const vector_t &inputDesired);

    Task formulateMRAConstraints(const vector_t &stateDesired, const vector_t &inputDesired);

    vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode, scalar_t period);

private:
    scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
    Vector6 alpha_hat, beta_hat, delta_hat, gamma;
    matrix_t K_alpha, Q_alpha, H;
    scalar_t P_adaptive, epsilon_theta, theta_max, lambda;
};

} // namespace legged

#endif // LEG_WBC_MRA_WBC_H
