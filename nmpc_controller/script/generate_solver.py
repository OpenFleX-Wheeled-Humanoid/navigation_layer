#!/usr/bin/env python3
"""
Generate acados NMPC solver C code for swerve drive navigation.

State:   x = [px, py, theta, vx, vy, omega]  (6)
Control: u = [ax, ay, alpha]                   (3)
Params:  p = [xref(6), costmap_cost(1)]        (7 per stage)

Usage:
    cd src/nmpc_controller
    python3 script/generate_solver.py
"""

import os
import shutil
import numpy as np
from casadi import SX, vertcat, cos, sin
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

MODEL_NAME = "swerve_model"

# Horizon
N = 20
TF = 2.0

# Bounds
VX_MAX = 0.5
VX_MIN = -0.3
VY_MAX = 0.5
OMEGA_MAX = 1.0
AX_MAX = 2.0
AY_MAX = 2.0
ALPHA_MAX = 3.0

# Cost weights
Q_DIAG = [15.0, 15.0, 10.0, 1.0, 1.0, 0.5]  # state tracking
R_DIAG = [0.1, 0.1, 0.05]                      # control effort
COSTMAP_W = 50.0                                 # obstacle avoidance
Q_E_DIAG = [30.0, 30.0, 20.0, 1.0, 1.0, 0.5]  # terminal


def create_model() -> AcadosModel:
    model = AcadosModel()
    model.name = MODEL_NAME

    # States
    px = SX.sym("px")
    py = SX.sym("py")
    theta = SX.sym("theta")
    vx = SX.sym("vx")
    vy = SX.sym("vy")
    omega = SX.sym("omega")
    x = vertcat(px, py, theta, vx, vy, omega)

    # Controls
    ax = SX.sym("ax")
    ay = SX.sym("ay")
    alpha = SX.sym("alpha")
    u = vertcat(ax, ay, alpha)

    # Parameters: reference state (6) + costmap cost (1)
    xref = SX.sym("xref", 6)
    costmap_cost = SX.sym("costmap_cost")
    p = vertcat(xref, costmap_cost)

    # Continuous dynamics (body-frame velocities -> world-frame)
    dpx = vx * cos(theta) - vy * sin(theta)
    dpy = vx * sin(theta) + vy * cos(theta)
    dtheta = omega
    dvx = ax
    dvy = ay
    domega = alpha
    f_expl = vertcat(dpx, dpy, dtheta, dvx, dvy, domega)

    # State derivative placeholder
    xdot = SX.sym("xdot", 6)

    model.x = x
    model.u = u
    model.xdot = xdot
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    # Cost residuals (NONLINEAR_LS)
    # Stage: y = [x - xref; u; costmap_cost]  (10-dim)
    model.cost_y_expr = vertcat(x - xref, u, costmap_cost)
    # Terminal: y_e = [x - xref]  (6-dim)
    model.cost_y_expr_e = x - xref

    return model


def create_ocp() -> AcadosOcp:
    ocp = AcadosOcp()
    model = create_model()
    ocp.model = model

    nx = 6
    nu = 3
    ny = nx + nu + 1   # 10: stage residual dimension
    ny_e = nx           # 6: terminal residual dimension
    np_val = 7          # parameter dimension

    ocp.solver_options.N_horizon = N
    ocp.dims.np = np_val

    # Solver options
    ocp.solver_options.tf = TF
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_max_iter = 1
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.print_level = 0

    # Cost type
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    # Stage cost weight matrix W (10x10)
    W = np.diag(Q_DIAG + R_DIAG + [COSTMAP_W])
    ocp.cost.W = W

    # Terminal cost weight matrix W_e (6x6)
    W_e = np.diag(Q_E_DIAG)
    ocp.cost.W_e = W_e

    # Reference (will be set at runtime)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # Parameter defaults
    ocp.parameter_values = np.zeros(np_val)

    # Control constraints
    ocp.constraints.lbu = np.array([-AX_MAX, -AY_MAX, -ALPHA_MAX])
    ocp.constraints.ubu = np.array([AX_MAX, AY_MAX, ALPHA_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2])

    # State constraints (velocity bounds only, indices 3,4,5)
    ocp.constraints.lbx = np.array([VX_MIN, -VY_MAX, -OMEGA_MAX])
    ocp.constraints.ubx = np.array([VX_MAX, VY_MAX, OMEGA_MAX])
    ocp.constraints.idxbx = np.array([3, 4, 5])

    # Initial state constraint (all 6 states)
    ocp.constraints.x0 = np.zeros(nx)

    # Code generation settings
    ocp.code_gen_opts.code_export_directory = "generated"

    return ocp


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    gen_dir = os.path.join(pkg_dir, "generated")

    # Clean previous generated code
    if os.path.exists(gen_dir):
        shutil.rmtree(gen_dir)
    os.makedirs(gen_dir, exist_ok=True)

    # Change to package directory for code generation
    orig_dir = os.getcwd()
    os.chdir(pkg_dir)

    try:
        ocp = create_ocp()
        solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
        print(f"[generate_solver] Solver generated successfully in {gen_dir}")
        print(f"[generate_solver] N={N}, Tf={TF}, nx=6, nu=3")

        # Quick sanity check
        status = solver.solve()
        print(f"[generate_solver] Sanity solve status: {status} (0=success)")
    finally:
        os.chdir(orig_dir)

    # Clean up json file
    json_path = os.path.join(pkg_dir, "acados_ocp.json")
    if os.path.exists(json_path):
        # Keep it for reference but move to generated/
        shutil.move(json_path, os.path.join(gen_dir, "acados_ocp.json"))


if __name__ == "__main__":
    main()
