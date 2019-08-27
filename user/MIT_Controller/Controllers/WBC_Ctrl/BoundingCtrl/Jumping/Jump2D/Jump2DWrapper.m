%% ========================================================================
%
% Calculates all of the iterative symbolic equations for the 2D jump and
% creates optimized functions for each of them, ready to be converted to
% working C++ code.
%
% Functions that get created are:
%   - Bounds
%   - Initialize
%   - Cost
%   - Gradient
%   - Constraints
%   - Constraints Final
%   - Constraint Jacobian
%   - Constraint Jacobian Sparse Index
%   - Constraint Jacobian Final
%   - Constraint Jacobian Final Sparse Index
%   - Constraint Jacobian Initial
%   - Constraint Jacobian Initial Sparse Index
%   - Lagrangian Hessian
%   - Lagrangian Hessian Sparse Index
%   - Lagrangian Hessian Final
%   - Lagrangian Hessian Final Sparse Index
%   - Lagrangian Hessian Initial
%   - Lagrangian Hessian Initial Sparse Index

%% Parameter Initialization

%% Parameter setup
% Number of feet (front 2 and back 2 are paired together)
NUM_FEET = 2;

% Number of timesteps
N = 1;

% Decision Variables (for current timestep)
x = 0;
z = 0;
phi = 0;
dx = 0;
dz = 0;
dphi = 0;
Fxf = 0;
Fzf = 0;
Fxb = 0;
Fzb = 0;
states = [x;z;phi;dx;dz;dphi];
inputs = [Fxf;Fzf;Fxb;Fzb];

% Decision Variables (for previous timestep)
x0 = 0;
z0 = 0;
phi0 = 0;
dx0 = 0;
dz0 = 0;
dphi0 = 0;
Fxf0 = 0;
Fzf0 = 0;
Fxb0 = 0;
Fzb0 = 0;
states0 = [x0;z0;phi0;dx0;dz0;dphi0];
inputs0 = [Fxf0;Fzf0;Fxb0;Fzb0];

% Desired states and inputs
x_d = 0;
z_d = 0;
phi_d = 0;
dx_d = 0;
dz_d = 0;
dphi_d = 0;
Fxf_d = 0;
Fzf_d = 0;
Fxb_d = 0;
Fzb_d = 0;
states_ref = [x_d;z_d;phi_d;dx_d;dz_d;dphi_d];
inputs_ref = [Fxf_d;Fzf_d;Fxb_d;Fzb_d];

% Decision Variable minimums
x_min = 0;
z_min = 0;
phi_min = 0;
dx_min = 0;
dz_min = 0;
dphi_min = 0;
Fxf_min = 0;
Fzf_min = 0;
Fxb_min = 0;
Fzb_min = 0;
states_min = [x_min;z_min;phi_min;dx_min;dz_min;dphi_min];
inputs_min = [Fxf_min;Fzf_min;Fxb_min;Fzb_min];

% Decision Variable maximums
x_max = 0;
z_max = 0;
phi_max = 0;
dx_max = 0;
dz_max = 0;
dphi_max = 0;
Fxf_max = 0;
Fzf_max = 0;
Fxb_max = 0;
Fzb_max = 0;
states_max = [x_max;z_max;phi_max;dx_max;dz_max;dphi_max];
inputs_max = [Fxf_max;Fzf_max;Fxb_max;Fzb_max];

% Footstep variables
rxf = 0;
rzf = 0;
rxb = 0;
rzb = 0;
r_foot = [rxf;rzf;rxb;rzb];

% Contact Variables
sf = 0;
sb = 0;
c_states = [sf;sb];

% Sparsity pattern parameters
iter = 0;
NUM_X = 0;
NUM_C = 0;

% Optimization Parameters
NUM_STATES = size(states,1);
NUM_INPUTS = size(inputs,1);

% Gravity and timestep
g = 0;
dt = 0;
mu_g = 0;

% Physical robot Parameters
m = 0;
Iyy = 0;
leg_length_max = 0;

% Cost Function Weights
Q = zeros(NUM_STATES,1);  % states
R = zeros(NUM_INPUTS,1);  % inputs

% Hessian objective factor
obj_factor = 0;


%% Iterative Functions
% Initialization

% Generate decision variable and constraint bounds
[decision_vars_lb, decision_vars_ub, constraints_ub, constraints_lb] = ...
    Jump2DBounds(states_max, states_min, inputs_max, inputs_min, c_states);

% Generate initial guess, reference policy, and footstep references
[decision_vars0, inputs_ref0] = ...
    Jump2DInitialize(states0, inputs, c_states, dt, r_foot, m, Iyy, g);

% Cost function

% Generate cost function
J = ...
    Jump2DCost(states, inputs, states_ref, inputs_ref, Q, R);


% Cost gradient

% Generate cost gradient function
cost_gradient = ...
    Jump2DCostGradient(states, inputs, states_ref, inputs_ref, Q, R);


% Constraints

% Generate constraints function
constraints = ...
    Jump2DConstraints(states, inputs, states0, c_states, dt,...
    r_foot, m, Iyy, g, mu_g);

% Generate final constraints function
constraints_final = ...
    Jump2DConstraintsFinal(states, inputs, states0, c_states, dt,...
    r_foot, m, Iyy, g, mu_g);

% Generate initial constraints function
constraints_initial = ...
    Jump2DConstraintsInitial(states, inputs, states0, c_states, dt,...
    r_foot, m, Iyy, g, mu_g);


% Constraint Jacobian

% Generate constraint jacobian functionconstraint_jacobian_nz = ...
Jump2DConstraintJacobian(c_states, dt, r_foot, m, Iyy, mu_g);

% Generate constraint jacobian sparsity function
[row_index_CJ, col_index_CJ] = ...
    Jump2DConstraintJacobianSP(iter, NUM_X, NUM_C);

% Generate final constraint jacobian function
constraint_jacobian_final_nz = ...
    Jump2DConstraintJacobianFinal(c_states, dt, r_foot, m, Iyy, mu_g);

% Generate final constraint jacobian sparsity function
[row_index_final_CJ, col_index_final_CJ] = ...
    Jump2DConstraintJacobianFinalSP(iter, NUM_X, NUM_C);

% Generate initial constraint jacobian function
constraint_jacobian_initial_nz = ...
    Jump2DConstraintJacobianInitial(c_states, dt, r_foot, m, Iyy, mu_g);

% Generate initial constraint jacobian sparsity function
[row_index_initial_CJ, col_index_initial_CJ] = ...
    Jump2DConstraintJacobianInitialSP(iter, NUM_X, NUM_C);


% Hessian

% Generate lagrangian hessian function
lagrangian_hessian_nz = ...
    Jump2DLagrangianHessian(Q, R, obj_factor);

% Generate hessian sparsity function
[row_index_H, col_index_H] = ...
    Jump2DLagrangianHessianSP(iter, NUM_X);

% Generate final lagrangian hessian function
lagrangian_hessian_final_nz = ...
    Jump2DLagrangianHessianFinal(Q, R, obj_factor);

% Generate final hessian sparsity function
[row_index_final_H, col_index_final_H] = ...
    Jump2DLagrangianHessianFinalSP(iter, NUM_X);

% Generate initial lagrangian hessian function
lagrangian_hessian_initial_nz = ...
    Jump2DLagrangianHessianInitial(Q, R, obj_factor);

% Generate initial hessian sparsity function
[row_index_initial_H, col_index_initial_H] = ...
    Jump2DLagrangianHessianInitialSP(iter, NUM_X);

