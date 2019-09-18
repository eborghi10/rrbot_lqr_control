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

  Xg << M_PI, 0, 0, 0;

  return true;
}

void LQRController::update(const ros::Time& time, const ros::Duration& period)
{
  // Mechanical properties
  const double l1 = 2;
  const double l2 = 2;
  const double lc1 = l1 / 2.;
  const double lc2 = l2 / 2.;
  const double l1_sq = l1 * l1;
  const double l2_sq = l2 * l2;
  const double m1 = 1;
  const double m2 = 1;
  const double g = 9.80665;

  // Moment of inertia of a link
  const double I1 = m1 / 12.0 * l1_sq;
  const double I2 = m2 / 12.0 * l2_sq;

  // Dynamical constants
  const double a = I1 + m2 * l1_sq;
  const double b = I2;
  const double c = m2 * l1 * lc2;
  const double d = g * (m1 * lc1 + m2 * l1);
  const double e = g * m2 * lc2;

  // Angles
  const double th1 = passive_joint_.getPosition();
  const double th2 = joint_.getPosition();
  const double dth1 = passive_joint_.getVelocity();
  const double dth2 = joint_.getVelocity();

  Eigen::Matrix<double, 4, 1> X;
  X << th1, th2, dth1, dth2;

  M(0,0) = a + b + 2 * c * std::cos(th2);
  M(0,1) = b + c * std::cos(th2);
  M(1,0) = M(0,1);
  M(1,1) = b;

  // C = 0 in the upright point
  // C(0,0) = -c * std::sin(th2) * dth2;
  // C(0,1) = -c * std::sin(th2) * (dth1 + dth2);
  // C(1,0) = c * std::sin(th2) * dth1;
  // C(1,1) = 0;

  G(0) = -d * std::sin(th1) - e * std::sin(th1 + th2);
  G(1) = -e * std::sin (th1 + th2);

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
  A.block<2,2>(2,0) = M.inverse() * dG_dQ;
  // A.block<2,2>(3,3) = -M.inverse() * C;

  B.segment<2>(2) = M.inverse() * B_;

  Q(0,0) = 10.0;
  Q(1,1) = 10.0;
  Q(2,2) = 0.1;
  Q(3,3) = 0.1;

  R << 0.01;

  // https://github.com/wiany11/intelligent_robotics__lqr_wip/blob/master/ctrl/dynamics.py#L46
  const Eigen::Matrix<double, 1, 4> K = calcGainK();

  Eigen::Matrix<double, 4, 1> U = Xg - X;

  const double KU = -K * U;

  joint_.setCommand(KU);
}

} // namespace