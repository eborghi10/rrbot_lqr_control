#include <rrbot_control/lqr_controller.hpp>

namespace controller_ns{

bool LQRController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{
  // get joint name from the parameter server
  std::string my_joint;
  if (!n.getParam("joint", my_joint)){
    ROS_ERROR("Could not find joint name");
    return false;
  }

  std::string passive_joint;
  if (!n.getParam("underactuated", passive_joint)){
    ROS_ERROR("Could not find underactuated name");
    return false;
  }

  // get the joint object to use in the realtime loop
  joint_ = hw->getHandle(my_joint);  // throws on failure
  passive_joint_ = hw->getHandle(passive_joint);
  return true;
}

void LQRController::update(const ros::Time& time, const ros::Duration& period)
{
  // sudo apt-get install -y python-catkin-tools libeigen3-dev libboost-all-dev
  // rosdep install --from-path src -yi
  // catkin build -DCMAKE_BUILD_TYPE=Release -j2
  // https://ethz-adrl.github.io/ct/ct_doc/doc/html/optcon_tut_lqr.html
  // https://ethz-adrl.github.io/ct/ct_doc/doc/html/tut_basics.html#tut_basics_pkg
  // // get the state and control input dimension of the oscillator
  // const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
  // const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;
  // // create an auto-differentiable instance of the oscillator dynamics
  // ct::core::ADCGScalar w_n(50.0);
  // std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim, ct::core::ADCGScalar>> oscillatorDynamics(
  //     new ct::core::tpl::SecondOrderSystem<ct::core::ADCGScalar>(w_n));
  // // create an Auto-Differentiation Linearizer with code generation on the quadrotor model
  // ct::core::ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorDynamics);
  // // compile the linearized model just-in-time
  // adLinearizer.compileJIT();
  // // define the linearization point around steady state
  // ct::core::StateVector<state_dim> x;
  // x.setZero();
  // ct::core::ControlVector<control_dim> u;
  // u.setZero();
  // double t = 0.0;
  // // compute the linearization around the nominal state using the Auto-Diff Linearizer
  // auto A = adLinearizer.getDerivativeState(x, u, t);
  // auto B = adLinearizer.getDerivativeControl(x, u, t);
  // // load the weighting matrices
  // Eigen::MatrixXd q;
  // q(0, 0) = 1.6;
  // q(1,1) = 1.0;
  // Eigen::MatrixXd r;
  // r(0,0) = 1.0;
  // ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost(q, r);
  // // const std::string costFile = std::string(COSTFUNCTION_TEST_DIR) + "/lqrCost.info";
  // // quadraticCost.loadConfigFile(costFile, "termLQR");

  // auto Q = quadraticCost.stateSecondDerivative(x, u, t);    // x, u and t can be arbitrary here
  // auto R = quadraticCost.controlSecondDerivative(x, u, t);  // x, u and t can be arbitrary here
  
  // // design the LQR controller
  // ct::optcon::LQR<state_dim, control_dim> lqrSolver;
  // ct::core::FeedbackMatrix<state_dim, control_dim> K;
  // lqrSolver.compute(Q, R, A, B, K);
  // ROS_INFO_STREAM(passive_joint_.getPosition());
  // double error = setpoint_ - passive_joint_.getPosition();
  // joint_.setCommand(error*gain_);

  // Mechanical properties
  const double l1 = 2;
  const double l2 = 2;
  const double l1_sq = l1 * l1;
  const double l2_sq = l2 * l2;
  const double m1 = 1;
  const double m2 = 1;
  const double g = 9.80665;

  // Moment of inertia of a link
  const double I1 = m1 / 12.0 * l1_sq;
  const double I2 = m2 / 12.0 * l2_sq;

  // Dynamical constants
  const double a = I1 + I2;
  const double b = I2;
  const double c = m2 * l1 * l2;
  const double d = g * (m1 * l1 + m2 * l1);
  const double e = g * m2 * l2;

  // Angles
  const double th1 = passive_joint_.getPosition();
  const double th2 = joint_.getPosition();
  const double dth1 = passive_joint_.getVelocity();
  const double dth2 = joint_.getVelocity();

  Eigen::Array4d X;
  X << th1, th2, dth1, dth2;

  H(0,0) = a + b + 2 * c * std::cos(th2);
  H(0,1) = b + c * std::cos(th2);
  H(1,0) = H(0,1);
  H(1,1) = b;

  // C = 0 in the upright point
  // C(0,0) = -c * std::sin(th2) * dth2;
  // C(0,1) = -c * std::sin(th2) * (dth1 + dth2);
  // C(1,0) = c * std::sin(th2) * dth1;
  // C(1,1) = 0;

  G(0) = d * std::sin(th1) + e * std::sin(th1 + th2);
  G(1) = e * std::sin (th1 + th2);

  B_(1) = 1;

  Eigen::Matrix2d dG_dQ;
  // dG(0)_dq1
  dG_dQ(0,0) = -d -e;
  // dG(0)_dq2
  dG_dQ(0,1) = -e;
  // dG(1)_dq1
  dG_dQ(1,0) = -e;
  // dG(1)_dq2
  dG_dQ(1,1) = -e;

  // Linearization

  A.block<2,2>(0,2) = Eigen::Matrix2d::Identity();
  A.block<2,2>(2,0) = -H.inverse() * dG_dQ;
  // A.block<2,2>(3,3) = -H.inverse() * C;

  B.segment<2>(2) = H.inverse() * B_;

  Q(0,0) = 1;
  Q(1,1) = 1;
  Q(2,2) = 10;
  Q(3,3) = 10;

  R(0,0) = 0.1;
  R(1,1) = 0.1;
  R(2,2) = 1;
  R(3,3) = 1;

  // https://github.com/wiany11/intelligent_robotics__lqr_wip/blob/master/ctrl/dynamics.py#L46
  Eigen::MatrixXd K;
  K << calcGainK();

  // const Eigen::MAtrixXd X_dot = (A - B * K).dot(X);
  // const double
  // joint_.setCommand(K(0));
}

}//namespace