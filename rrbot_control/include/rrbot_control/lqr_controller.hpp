#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <ct/optcon/optcon.h>  // also includes ct_core
#include <memory>

#include <eigen3/Eigen/Dense>

// #define COSTFUNCTION_TEST_DIR "@COSTFUNCTION_TEST_DIR@"

namespace controller_ns{

class LQRController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:  
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time) { };
  void stopping(const ros::Time& time) { };
  void update(const ros::Time& time, const ros::Duration& period);

private:
  hardware_interface::JointHandle joint_;
  hardware_interface::JointHandle passive_joint_;

  // https://qiita.com/watakandhi/items/a020aec6d74e6dc7ef30

  Eigen::Matrix2d H;
  Eigen::Matrix2d C;
  Eigen::Array2d G;
  Eigen::Array2d B_;

  Eigen::Matrix4d A;
  Eigen::Array4d B;
  Eigen::Matrix4d Q;
  Eigen::Matrix4d R;

  Eigen::MatrixXd calcGainK()
  {
      /**
      * Calculate LQR Gain K
      * Solves Riccati Equation using Arimoto Potter Method
      * 
      * author: Horibe Takamasa
      */

      // https://www.mathworks.com/help/control/ref/lqr.html
      // http://www.kostasalexis.com/lqr-control.html
      Eigen::MatrixXd  P = care(A, B, Q, R);
      return R.inverse() * (B.transpose() * P);
  }

  Eigen::MatrixXd care(const Eigen::MatrixXd &A,
                       const Eigen::MatrixXd &B,
                       const Eigen::MatrixXd &Q,
                       const Eigen::MatrixXd &R)
  {
      const size_t dim_x = A.rows();
      
      // Set Hamilton Matrix
      Eigen::MatrixXd Ham(2*dim_x, 2*dim_x);
      Ham << A, -B*R.inverse()*B.transpose(), -Q, -A.transpose();

      // calc eigenvalues and eigenvectors
      Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);
      if (Eigs.info() != Eigen::Success) abort();

      // extract stable eigenvectors into 'eigvec'
      Eigen::MatrixXcd eigvec(2*dim_x, dim_x);
      int j = 0;

      // store those with negative real number
      for(int i = 0; i < 2*dim_x; ++i){
          if(Eigs.eigenvalues()[i].real() < 0){
              eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2*dim_x, 1);
              ++j;
          }
      }

      // calc P with stable eigen vector matrix
      Eigen::MatrixXcd U(dim_x, dim_x);
      Eigen::MatrixXcd V(dim_x, dim_x);

      U = eigvec.block(0,0,dim_x,dim_x);
      V = eigvec.block(dim_x,0,dim_x,dim_x);

      return (V * U.inverse()).real();
  }
};

PLUGINLIB_EXPORT_CLASS(controller_ns::LQRController, controller_interface::ControllerBase);

}//namespace

#endif // LQR_CONTROLLER_HPP
