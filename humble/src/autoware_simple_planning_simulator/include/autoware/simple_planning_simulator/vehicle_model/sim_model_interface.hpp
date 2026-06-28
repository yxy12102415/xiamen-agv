#ifndef AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_
#define AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_

#include <Eigen/Core>

namespace autoware::simulator::simple_planning_simulator
{

class SimModelInterface
{
public:
  SimModelInterface(const int dim_x, const int dim_u)
  : dim_x_(dim_x), dim_u_(dim_u), state_(Eigen::VectorXd::Zero(dim_x)),
    input_(Eigen::VectorXd::Zero(dim_u))
  {
  }

  virtual ~SimModelInterface() = default;

  virtual double getX() = 0;
  virtual double getY() = 0;
  virtual double getYaw() = 0;
  virtual double getVx() = 0;
  virtual double getVy() = 0;
  virtual double getAx() = 0;
  virtual double getWz() = 0;
  virtual double getSteer() = 0;
  virtual void update(const double & dt) = 0;
  virtual Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) = 0;

  void setState(const Eigen::VectorXd & state) { state_ = state; }
  void setInput(const Eigen::VectorXd & input) { input_ = input; }

protected:
  void updateRungeKutta(const double & dt, const Eigen::VectorXd & input)
  {
    const Eigen::VectorXd k1 = calcModel(state_, input);
    const Eigen::VectorXd k2 = calcModel(state_ + 0.5 * dt * k1, input);
    const Eigen::VectorXd k3 = calcModel(state_ + 0.5 * dt * k2, input);
    const Eigen::VectorXd k4 = calcModel(state_ + dt * k3, input);
    state_ += dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
  }

  int dim_x_;
  int dim_u_;
  Eigen::VectorXd state_;
  Eigen::VectorXd input_;
};

}  // namespace autoware::simulator::simple_planning_simulator

#endif  // AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_
