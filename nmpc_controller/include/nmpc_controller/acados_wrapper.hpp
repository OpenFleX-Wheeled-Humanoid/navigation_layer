#ifndef NMPC_CONTROLLER__ACADOS_WRAPPER_HPP_
#define NMPC_CONTROLLER__ACADOS_WRAPPER_HPP_

#include <array>
#include <memory>
#include <string>

// acados C headers
extern "C" {
#include "acados/utils/types.h"
#include "acados_c/ocp_nlp_interface.h"
}

namespace nmpc_controller
{

// Dimensions matching generate_solver.py
constexpr int NX = 6;   // px, py, theta, vx, vy, omega
constexpr int NU = 3;   // ax, ay, alpha
constexpr int NP = 7;   // xref(6) + costmap_cost(1)
constexpr int NY = 10;  // state_err(6) + control(3) + costmap(1)
constexpr int NY_E = 6; // terminal state error

using StateVec = std::array<double, NX>;
using ControlVec = std::array<double, NU>;
using ParamVec = std::array<double, NP>;
using StageRefVec = std::array<double, NY>;
using TerminalRefVec = std::array<double, NY_E>;

class AcadosWrapper
{
public:
  AcadosWrapper();
  ~AcadosWrapper();

  // Prevent copy
  AcadosWrapper(const AcadosWrapper &) = delete;
  AcadosWrapper & operator=(const AcadosWrapper &) = delete;

  bool initialize(int N, double Tf);
  void cleanup();

  int getHorizonSteps() const { return N_; }

  void setInitialState(const StateVec & x0);
  void setStateGuess(int stage, const StateVec & x);
  void setControlGuess(int stage, const ControlVec & u);

  void setStageReference(int stage, const StageRefVec & yref);
  void setTerminalReference(const TerminalRefVec & yref_e);

  void setStageParameters(int stage, const ParamVec & p);

  void setStageWeights(const std::array<double, NY> & W_diag);
  void setTerminalWeights(const std::array<double, NY_E> & W_e_diag);

  void setControlBounds(double ax_max, double ay_max, double alpha_max);
  void setStateBounds(double vx_min, double vx_max, double vy_max, double omega_max);

  int solve();

  ControlVec getControl(int stage) const;
  StateVec getState(int stage) const;

  void shiftSolution();

  double getSolveTimeMs() const;

private:
  // acados generated API capsule (forward-declared, actual type from generated code)
  void * capsule_{nullptr};

  ocp_nlp_config * nlp_config_{nullptr};
  ocp_nlp_dims * nlp_dims_{nullptr};
  ocp_nlp_in * nlp_in_{nullptr};
  ocp_nlp_out * nlp_out_{nullptr};
  ocp_nlp_solver * nlp_solver_{nullptr};

  int N_{0};
  bool initialized_{false};
  double solve_time_ms_{0.0};
};

}  // namespace nmpc_controller

#endif  // NMPC_CONTROLLER__ACADOS_WRAPPER_HPP_
