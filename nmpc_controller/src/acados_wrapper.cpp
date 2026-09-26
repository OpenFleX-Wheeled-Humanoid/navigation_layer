#include "nmpc_controller/acados_wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

// Generated solver headers
extern "C" {
#include "acados_solver_swerve_model.h"
}

namespace nmpc_controller
{

AcadosWrapper::AcadosWrapper() = default;

AcadosWrapper::~AcadosWrapper()
{
  cleanup();
}

bool AcadosWrapper::initialize(int N, double /*Tf*/)
{
  if (initialized_) {
    cleanup();
  }

  N_ = N;

  // Create acados solver capsule
  auto * cap = swerve_model_acados_create_capsule();
  if (!cap) {
    return false;
  }

  int status = swerve_model_acados_create(cap);
  if (status != 0) {
    swerve_model_acados_free_capsule(cap);
    return false;
  }

  capsule_ = cap;
  nlp_config_ = swerve_model_acados_get_nlp_config(cap);
  nlp_dims_ = swerve_model_acados_get_nlp_dims(cap);
  nlp_in_ = swerve_model_acados_get_nlp_in(cap);
  nlp_out_ = swerve_model_acados_get_nlp_out(cap);
  nlp_solver_ = swerve_model_acados_get_nlp_solver(cap);

  initialized_ = true;
  return true;
}

void AcadosWrapper::cleanup()
{
  if (initialized_ && capsule_) {
    auto * cap = static_cast<swerve_model_solver_capsule *>(capsule_);
    swerve_model_acados_free(cap);
    swerve_model_acados_free_capsule(cap);
    capsule_ = nullptr;
    initialized_ = false;
  }
}

void AcadosWrapper::setInitialState(const StateVec & x0)
{
  // Set x0 as equality constraint at stage 0
  // Signature: ocp_nlp_constraints_model_set(config, dims, in, out, stage, field, value)
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", const_cast<double *>(x0.data()));
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", const_cast<double *>(x0.data()));
}

void AcadosWrapper::setStateGuess(int stage, const StateVec & x)
{
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, stage, "x", const_cast<double *>(x.data()));
}

void AcadosWrapper::setControlGuess(int stage, const ControlVec & u)
{
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, stage, "u", const_cast<double *>(u.data()));
}

void AcadosWrapper::setStageReference(int stage, const StageRefVec & yref)
{
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "yref", const_cast<double *>(yref.data()));
}

void AcadosWrapper::setTerminalReference(const TerminalRefVec & yref_e)
{
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "yref", const_cast<double *>(yref_e.data()));
}

void AcadosWrapper::setStageParameters(int stage, const ParamVec & p)
{
  swerve_model_acados_update_params(
    static_cast<swerve_model_solver_capsule *>(capsule_),
    stage, const_cast<double *>(p.data()), NP);
}

void AcadosWrapper::setStageWeights(const std::array<double, NY> & W_diag)
{
  // Set diagonal weight matrix W for stages 0..N-1
  std::array<double, NY * NY> W{};
  for (int i = 0; i < NY; i++) {
    W[i * NY + i] = W_diag[i];
  }
  for (int k = 0; k < N_; k++) {
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "W", W.data());
  }
}

void AcadosWrapper::setTerminalWeights(const std::array<double, NY_E> & W_e_diag)
{
  std::array<double, NY_E * NY_E> W_e{};
  for (int i = 0; i < NY_E; i++) {
    W_e[i * NY_E + i] = W_e_diag[i];
  }
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "W", W_e.data());
}

void AcadosWrapper::setControlBounds(double ax_max, double ay_max, double alpha_max)
{
  std::array<double, NU> lbu = {-ax_max, -ay_max, -alpha_max};
  std::array<double, NU> ubu = {ax_max, ay_max, alpha_max};
  for (int k = 0; k < N_; k++) {
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, k, "lbu", lbu.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, k, "ubu", ubu.data());
  }
}

void AcadosWrapper::setStateBounds(double vx_min, double vx_max, double vy_max, double omega_max)
{
  std::array<double, 3> lbx = {vx_min, -vy_max, -omega_max};
  std::array<double, 3> ubx = {vx_max, vy_max, omega_max};
  // State bounds on stages 1..N (stage 0 is initial state constraint)
  for (int k = 1; k <= N_; k++) {
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, k, "lbx", lbx.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, k, "ubx", ubx.data());
  }
}

int AcadosWrapper::solve()
{
  auto start = std::chrono::steady_clock::now();
  int status = swerve_model_acados_solve(static_cast<swerve_model_solver_capsule *>(capsule_));
  auto end = std::chrono::steady_clock::now();
  solve_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
  return status;
}

ControlVec AcadosWrapper::getControl(int stage) const
{
  ControlVec u{};
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
  return u;
}

StateVec AcadosWrapper::getState(int stage) const
{
  StateVec x{};
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());
  return x;
}

void AcadosWrapper::shiftSolution()
{
  // Shift states: x[k] = x[k+1] for k=0..N-2, x[N-1] = x[N]
  for (int k = 0; k < N_; k++) {
    StateVec x_next = getState(k + 1);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, k, "x", x_next.data());
  }
  // Shift controls: u[k] = u[k+1] for k=0..N-2, u[N-1] = u[N-1] (repeat last)
  for (int k = 0; k < N_ - 1; k++) {
    ControlVec u_next = getControl(k + 1);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, k, "u", u_next.data());
  }
}

double AcadosWrapper::getSolveTimeMs() const
{
  return solve_time_ms_;
}

}  // namespace nmpc_controller
