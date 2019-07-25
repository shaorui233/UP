% function SymbolicPRMPC

% Current Assumptions:
%   - Small pitch and roll in dynamics
%   - 1st decision variable state is current state (need to change this)
%   - Input decision variables are average over timestep
%   - Constraints are dependent on current and future state
%   - Cost is not dynamics dependent

% 0 for regularization R on initial

addpath Utils
clear; clc;

% Options for generating the symbolic functions
INITIALIZATION = true;
COST = false;
CONSTRAINTS = false;
LAGRANGIAN_HESSIAN = false;
GENERATE_FUNCTIONS = true;

%% General RPC Parameters
SymbolicPRMPC_timer = tic;
params_timer = tic;
fprintf('General symbolic variable setup...\n')

% Number of robot legs
NUM_FEET = 4;

% Problem Parameters
NUM_STATES = 12;
NUM_INPUTS = 6*NUM_FEET;
NUM_DECISION_VARS = NUM_STATES + NUM_INPUTS;
% {Dynamics} + {Foot on Ground} + {No Slip} + {Friction Pyramid} %+ {Kinematic Limits}
NUM_CONSTRAINTS = NUM_STATES + NUM_FEET + 2*NUM_FEET + 4*NUM_FEET; %+ NUM_FEET

% Symbolic parameters
syms dt real

% Sparsity pattern parameters
syms iter NUM_X NUM_C real

% Prediction steps
N = 1; K = 1; NK = N*K;

% Previous State vector
x0 = sym('x0_%d', [NUM_STATES, 1], 'real');

% State vector
x =  sym('x_%d', [NUM_STATES, 1], 'real');

% Vector from COM to foot position
r = sym('r%d_%d', [NUM_FEET, 3], 'real');

% Force Vector for each foot
f = sym('f%d_%d', [NUM_FEET, 3], 'real');

% Order the input vector U
u = [r, f]'; u = u(:);

% Desired Trajectory
x_d =  sym('x_d_%d', [NUM_STATES, 1], 'real');

% Nominal reference Input
u_ref =  sym('u_ref_%d', [NUM_INPUTS, 1], 'real');

% Reference input gains
K_ref = sym('K_ref_%d', [10, 1], 'real');

% Contacts state for the feet
c_state = sym('s%d', [NUM_FEET, 1], 'real');

% Weights states
Q =  diag(sym('q%d', [NUM_STATES, 1], 'real'));

% Weights input
R =  diag(sym('r%d', [NUM_INPUTS, 1], 'real'));

% Gravity vector
g = sym('g_%d', [3, 1], 'real');

% Robot physical parameters
m = sym('m', 'real');
I = sym('I_%d', [3, 1], 'real');
I_tensor = [m*eye(3), zeros(3); zeros(3), diag(I)];
I_tensor_inv = I_tensor^-1;

% Robot linearized system
System.A = [eye(6), dt*eye(6); zeros(6), eye(6)];
System.B = [(dt^2)/2*I_tensor_inv; dt*I_tensor_inv];
System.C = eye(12);
System.D = zeros(12,6);
System.Dw = [((dt)^2)/2*eye(6); eye(6)*dt];
System.w = [g;0;0;0];

fprintf('DONE general symbolic variable setup: %f s\n\n',toc(params_timer));


%% Initialization
if INITIALIZATION
    % Start the timer for calculating constraints
    initialization_timer = tic;
    fprintf('Calculating Initialization...\n')
    0
    % Set the infinity value
    INF = 2e19;
    
    % General cyclic gait parameters
    syms stance_fraction flight_phase real
    
    % Maximum and minimum states
    x_max =  sym('x_max_%d', [12, 1], 'real');
    x_min =  sym('x_min_%d', [12, 1], 'real');
    
    % Maximum and minimum inputs
    u_max =  sym('u_max_%d', [24, 1], 'real');
    u_min =  sym('u_min_%d', [24, 1], 'real');
    
    % Hip locations relative to CoM in body frame
    r_hip =  sym('r_hip%d_%d', [NUM_FEET,3], 'real')';
    
    % Initial foot location
    p_foot0 =  sym('p_foot0%d_%d', [NUM_FEET,3], 'real')';
    
    % Stance time at current prediction
    T_s = sym('T_s%d', [NUM_FEET,1], 'real');
    
    % Boolean for each leg flagging touchdown event steps
    touchdown =  sym('touchdown%d', [NUM_FEET,1], 'real')';
    
    % CoM state at which a touchdown occurs for each leg
    x_touchdown =  sym('x_touchdown%d_%d', [NUM_FEET,3], 'real')';
    
    % Initialize reference foot locations
    p_ref = sym(zeros(3,NUM_FEET));
    
    for k = 1:NK
        1
        %%% STATE BOUNDS %%%
        % State upper bound
        state_ub = [x_max(1:2) + x_d(1:2); x_max(3:12)];
        
        % State lower bound
        state_lb = [x_min(1:2) + x_d(1:2); x_min(3:12)];
        
        % Orientation at beginning of the phase, yaw only
        rpy0 = [0;0;1].*x0(4:6);
        
        % Set the rotation matrix using rpy
        RotM = simplify(QuaternionMatrixSym(RPYToQuaternion(rpy0)'));
        RotYaw = simplify(QuaternionMatrixSym(RPYToQuaternion([0;0;rpy0(3)])'));
        
        % Sum of the number of feet in stance at current phase
        num_stance_feet = sum(c_state);
        
        % Normalized gravity
        norm_g = simplify(norm(g));
        
        % Normalized velocity
        norm_vel_d = simplify(norm(x_d(7:8)));
        
        % Set the bounds for each foot
        for foot = 1:NUM_FEET
            2
            % foot state number
            n = 6*(foot - 1);
                        
            %%% REFERENCE POLICY INPUTS %%%
            % Reference policy for foot touchdown position
            p_ref_touchdown = K_ref(1)*RotM*r_hip(1:3,foot)...                             % Hip location
                + K_ref(2)*T_s(foot)/2*x_d(7:9)...                                         % Raibert Heuristic
                + K_ref(3)*T_s(foot)/2*CrossProd([0;0;x_d(12)])*(RotM*r_hip(1:3,foot))...  % Modified Raibert Heuristic
                + K_ref(4)*sqrt(x0(3)/norm_g)*(x0(7:9) - x_d(7:9))...                      % Capture Point
                + K_ref(5)*[0,I(2),0;-I(1),0,0;0,0,0]*(x0(10:12) - x_d(10:12))...          % Angular rate compensation
                + K_ref(6)*x0(3)/norm_g*CrossProd([x_d(7:8);0])*[0;0;x_d(12)]...           % High Speed Turning
                + K_ref(7)*T_s(foot)/2*CrossProd([0;0;x_d(12)])*(RotM*r_hip(1:3,foot));% + zmp;    % Modified Raibert Heuristic

            % Determine if foot location needs to be updated
            p_ref(:,foot) = touchdown(foot)*p_ref_touchdown + ...
                (1 - touchdown(foot))*p_foot0(1:3,foot);
            
            % Find distance CoM has travelled since last foot touchdown
            travel_distance = (x0(1:3) + x0(7:9)*dt/2) - x_touchdown(:,foot);
            
            % Input reference policy for foot vector
            u_ref0(1 + n:3 + n, :) = c_state(foot)*(p_ref(:,foot) - ...
                [travel_distance(1:2);x0(3)]);% - (x0(1:3)));% + Kcoeff(leg)*xd(7:9))); add in some weighted x0 + xd blending
            
            % Input reference policy for feed forward forces
            % syms stance_fraction flight_phase
            u_ref0(4 + n:6 + n, :) = -c_state(foot)*(...
                K_ref(8)*stance_fraction*m*g/...                                % Vertical impulse scaling
                (flight_phase + (1-flight_phase)*num_stance_feet)...            % Avoid divide by zero
                + K_ref(9)*m*x_d(12)*norm_vel_d*[sin(x_d(6));-cos(x_d(6));0]);  % Turning feed forward
            
            %%% INPUT BOUNDS %%%
            % Foot vector input upper bound
            input_ub(1 + n:3 + n, :) = c_state(foot)*(RotM*r_hip(:,foot) + u_max(1 + n:3 + n));
            
            % Foot vector input lower bound
            input_lb(1 + n:3 + n, :) = c_state(foot)*(RotM*r_hip(:,foot) + u_min(1 + n:3 + n));
            
            % Ground reaction force input upper bound
            input_ub(4 + n:6 + n, :) = c_state(foot)*u_max(4 + n:6 + n);
            
            % Ground reaction force input lower bound
            input_lb(4 + n:6 + n, :) = c_state(foot)*u_min(4 + n:6 + n);
        end
        3
        %%% INITIAL STATES %%%
        % Convert GRF to momentum rate of change
        h_dot = NonlinearInput(x0(:,k), u_ref0(:,k), c_state);
        
        4
        % Compute simplified discrete dynamics
        y = simplify(DiscreteDynamics(x0(:,k), h_dot, System));
        
        5
        %%% CONSTRAINT BOUNDS %%%
        % Constraint index
        i_c = NUM_CONSTRAINTS*(k - 1);
        
        % Dynamics, equality constraint {NUM_STATES}
        constraint_ub(i_c + 1:i_c + NUM_STATES,1) = zeros(NUM_STATES,1);
        constraint_lb(i_c + 1:i_c + NUM_STATES,1) = zeros(NUM_STATES,1);
        i_c = i_c + NUM_STATES;
        
        % Foot on ground, equality constraint {NUM_FEET}
        constraint_ub(i_c + 1:i_c + NUM_FEET,1) = zeros(NUM_FEET,1);
        constraint_lb(i_c + 1:i_c + NUM_FEET,1) = zeros(NUM_FEET,1);
        i_c = i_c + NUM_FEET;
        
        % No slip, equality constraint {2*NUM_FEET}
        constraint_ub(i_c + 1:i_c + 2*NUM_FEET,1) = zeros(2*NUM_FEET,1);
        constraint_lb(i_c + 1:i_c + 2*NUM_FEET,1) = zeros(2*NUM_FEET,1);
        i_c = i_c + 2*NUM_FEET;
        
        % Friction pyramids, inequality constraint {4*NUM_FEET}
        constraint_ub(i_c + 1:i_c + 4*NUM_FEET,1) = zeros(4*NUM_FEET,1);
        constraint_lb(i_c + 1:i_c + 4*NUM_FEET,1) = -INF*ones(4*NUM_FEET,1);
        i_c = i_c + 4*NUM_FEET;
        
    end
    
    6
    % Initial reference inputs
    u_ref0 = simplify(u_ref0);
    
    % Initial decision variables
    X0 = [y; u_ref0];
    
    7
    % Decision variable bounds
    X_lb = simplify([state_lb;input_lb]);
    X_ub = simplify([state_ub;input_ub]);
    
    8
    % Reference footstep locations relative to CoM
    p_ref = simplify(p_ref);
    
    % Print timing statistics
    fprintf('DONE calculating initialization: %f s\n\n',toc(initialization_timer));
end


%% Cost, Gradient, and Cost Hessian
if COST
    % Start the timer for calculating cost
    cost_timer = tic;
    fprintf('Calculating Cost...\n')
    
    % Initialize cost
    cost = 0;
    
    % Run through one iteration of the cost calculation
    for k = 1:NK
        % Convert GRF to momentum rate of change
        h_dot = NonlinearInput(x(:,k), u(:,k), c_state);
        
        % Compute simplified discrete dynamics
        y = DiscreteDynamics(x(:,k), h_dot, System);
        
        % State trajectory tracking error
        x_error = (x_d - x(:,k));%y);
        
        % Policy reference regularization
        u_error = (u_ref - u(:,k));
        
        % Objective cost function
        cost = cost + x_error'*Q*x_error + u_error'*R*u_error;
    end
    
    % Cost
    J = simplify(cost);
    
    % Gradient
    fprintf('Calculating Cost Gradient...\n')
    cost_gradient = simplify(jacobian(J, [x;u])');
    
    % Cost Hessian
    fprintf('Calculating Cost Hessian...\n')
    syms obj_factor real
    cost_hessian = simplify(obj_factor*jacobian(cost_gradient, [x;u]));
    
    % Diagonal entries of weight matrices only for generation
    Q =  diag(Q); R =  diag(R);
    
    % Print timing statistics
    fprintf('DONE calculating cost function: %f s\n\n',toc(cost_timer));
end

%% Constraints
if CONSTRAINTS
    % Start the timer for calculating constraints
    constraint_timer = tic;
    fprintf('Calculating Constraints...\n')
    
    % Parameters
    syms mu_g real
    
    % State vector at future timestep
    xf = sym('xf_%d', [NUM_STATES, 1], 'real');
    
    % Vector from COM to foot position at future timestep
    rf = sym('rf%d_%d', [NUM_FEET, 3], 'real');
    
    % Force Vector for each foot at future timestep
    ff = sym('ff%d_%d', [NUM_FEET, 3], 'real');
    
    % Order the input vector u at future timestep
    uf = [rf, ff]'; uf = uf(:);
    
    % Contacts state for the feet at future timestep
    c_statef = sym('sf%d', [NUM_FEET, 1], 'real');
    
    % Ground height at each foot
    z_g =  sym('z_g%d', [NUM_FEET, 1], 'real');
    
    % Lagrange multipliers on the hessian
    lambda = sym('lambda%d', [NUM_CONSTRAINTS,1], 'real');
    
    % Initialize iteration constraint vector
    constraints = sym(zeros(NUM_CONSTRAINTS, NK));
    
    for k = 1:NK
        
        % Constraint index
        i_c = NUM_CONSTRAINTS*(k - 1);
        
        % Convert GRF to momentum rate of change
        h_dot = NonlinearInput(x(:,k), u(:,k), c_state);
        
        % Compute simplified discrete dynamics
        y = DiscreteDynamics(x(:,k), h_dot, System);
        
        % Dynamics, x_{k+1} - f(x_k, u_k) = 0 {NUM_STATES}
        constraints(i_c + 1:i_c + NUM_STATES,k) = xf(:,k) - y;
        i_c = i_c + NUM_STATES;
        
        % CoM postion in current step
        pos_com = x(1:3,k);
        
        % CoM postion in future step
        pos_comf = xf(1:3,k);
        
        % Add the constraints for each foot
        for foot = 1:NUM_FEET
            % Reset constraint index
            i_c = NUM_CONSTRAINTS*(k - 1) + NUM_STATES;
            
            % Leg state number
            n = 6*(foot - 1);
            
            % Foot position vector from CoM (CURRENTLY ONLY FOR FLAT GROUND)
            r_foot = u(1 + n: 3 + n);
            
            % Foot ground reaction forces
            f_foot = u(4 + n:6 + n);
            
            % Foot position in the world
            p_foot = (pos_com + r_foot);
            
            % Foot position vector from CoM at future timestep
            r_footf = uf(1 + n: 3 + n);
            
            % Foot position in the world at future timestep
            p_footf = (pos_comf + r_footf);
            
            % Error between foot positions over timestep
            p_foot_error = (p_footf - p_foot);
            
            %% Input constraints
            % Foot on ground {NUM_FEET}
            i_ground = i_c + (foot - 1);
            constraints(i_ground + 1,k) = ...
                c_state(foot)*(z_g(foot) - p_foot(3));
            i_c = i_c + NUM_FEET;
            
            % No slip {2*NUM_FEET}
            i_slip = i_c + 2*(foot - 1);
            constraints(i_slip + 1:i_slip + 2,k) = ...
                c_statef(foot)*c_state(foot)*p_foot_error(1:2);
            i_c = i_c + 2*NUM_FEET;
            
            % Friction pyramids {4*NUM_FEET}
            i_friction = i_c + 4*(foot - 1);
            constraints(i_friction + 1:i_friction + 4,k) = ...
                c_state(foot)*[f_foot(1) - mu_g*f_foot(3);...
                -f_foot(1) - mu_g*f_foot(3);...
                f_foot(2) - mu_g*f_foot(3);...
                -f_foot(2) - mu_g*f_foot(3)];
            
        end
    end
    
    % Simplify the symbolic constraints
    constraints = simplify(constraints);
    
    % Calculate the constraint jacobian
    %    rows: constraints
    %    columns: decision variables
    fprintf('Calculating Constraint Jacobian...\n')
    constraint_jacobian = simplify(jacobian(constraints,[x;u;xf;uf]));
    
    % Non zero entries in the constraint jacobian
    constraint_jacobian_nz = constraint_jacobian(constraint_jacobian~=0);
    
    % Calculate the constraint jacobian sparsity pattern
    fprintf('Calculating Constraint Jacobian Sparsity Pattern...\n')
    [row_index_CJ, col_index_CJ] = find(constraint_jacobian);
    
    % Shift indices for C++
    row_index_CJ = row_index_CJ - 1;
    col_index_CJ = col_index_CJ - 1;
    
    % Add the iteration index modifier
    row_index_CJ = row_index_CJ + ones(size(row_index_CJ,1),1)*iter*NUM_C;
    col_index_CJ = col_index_CJ + ones(size(col_index_CJ,1),1)*iter*NUM_X;
    
    % Calculate the constraint hessian
    fprintf('Calculating Constraint Hessian...\n')
    constraint_hessian = zeros(2*NUM_DECISION_VARS:2*NUM_DECISION_VARS);
    
    % Sum of the hessian of each constraint with Lagrange multiplier
    for c = 1:NUM_CONSTRAINTS
        constraint_hessian = constraint_hessian + ...
            lambda(c)*jacobian(constraint_jacobian(c,:),[x;u;xf;uf]);
    end
    
    % Non zeros only in current timestep entries
    constraint_hessian = simplify(constraint_hessian(1:NUM_DECISION_VARS,1:NUM_DECISION_VARS));
    
    %% Final constraints
    % Since the final timestep does not include future dynamics or foot
    % locations, it needs to be treated as a special case where only the
    % instantaneous forces and foot vectors are constrained.
    fprintf('Calculating Final Constraints...\n')
    
    % Final constraints don't include future decision variables
    constraints_final = constraints;
    
    % Final dynamics are not constrained
    constraints_final(1:NUM_STATES,1) = zeros(NUM_STATES,1);
    
    % Final no slip condition is not constrained
    i_slip = NUM_STATES + NUM_FEET;
    constraints_final(i_slip + 1:i_slip + 2*NUM_FEET,1) = ...
        zeros(2*NUM_FEET,1);
    
    % Final iteration constraint jacobian (no future decision variables)
    fprintf('Calculating Final Constraint Jacobian...\n')
    constraint_jacobian_final = simplify(jacobian(constraints_final,[x;u]));
    
    % Non zero entries in the final constraint jacobian
    constraint_jacobian_final_nz = ...
        constraint_jacobian_final(constraint_jacobian_final~=0);
    
    % Calculate the final constraint jacobian sparsity pattern
    fprintf('Calculating Final Constraint Jacobian Sparsity Pattern...\n')
    [row_index_final_CJ, col_index_final_CJ] = ...
        find(constraint_jacobian_final);
    
    % Shift indices for C++
    row_index_final_CJ = row_index_final_CJ - 1;
    col_index_final_CJ = col_index_final_CJ - 1;
    
    % Add the iteration index modifier
    row_index_final_CJ = row_index_final_CJ + ...
        ones(size(row_index_final_CJ,1),1)*iter*NUM_C;
    col_index_final_CJ = col_index_final_CJ + ...
        ones(size(col_index_final_CJ,1),1)*iter*NUM_X;
    
    % Calculate the final constraint hessian
    fprintf('Calculating Final Constraint Hessian...\n')
    constraint_hessian_final = zeros(NUM_DECISION_VARS:NUM_DECISION_VARS);
    
    % Sum of the hessian of each constraint with Lagrange multiplier
    for c = 1:NUM_CONSTRAINTS
        constraint_hessian_final = constraint_hessian_final + ...
            lambda(c)*jacobian(constraint_jacobian_final(c,:),[x;u]);
    end
    
    % Non zeros only in current timestep entries
    constraint_hessian_final = ...
        simplify(constraint_hessian_final(1:NUM_DECISION_VARS,1:NUM_DECISION_VARS));
    
    % Print out the non zero entry results
    fprintf('   Nonzeros in Constraint Jacobian: %i\n',nnz(constraint_jacobian));
    fprintf('   Nonzeros in final Constraint Jacobian: %i\n',nnz(constraint_jacobian_final));
    
    % Print timing statistics
    fprintf('DONE calculating constraints: %f s\n\n',toc(constraint_timer));
    
end


%% Lagrangian Hessian
if LAGRANGIAN_HESSIAN
    hessian_timer = tic;
    fprintf('Calculating Lagrangian Hessian...\n');
    
    % Combine the cost and constraint hessians
    lagrangian_hessian = cost_hessian + constraint_hessian;
    
    % Only use the lower triangular part since it is symmetric
    lagrangian_hessian_full = lagrangian_hessian;
    lagrangian_hessian = simplify(tril(lagrangian_hessian));
    
    % Non zero entries in the lagrangian hessian
    lagrangian_hessian_nz = lagrangian_hessian(lagrangian_hessian~=0);
    
    fprintf('Calculating Lagrangian Hessian Sparsity Pattern...\n')
    
    % Find the non-zero row and column indices
    [row_index_H, col_index_H] = find(lagrangian_hessian);
    
    % Shift indices for C++
    row_index_H = row_index_H - 1;
    col_index_H = col_index_H - 1;
    
    % Add the iteration index modifier
    row_index_H = row_index_H + ones(size(row_index_H,1),1)*iter*NUM_X;
    col_index_H = col_index_H + ones(size(col_index_H,1),1)*iter*NUM_X;
    
    %% Final Hessian
    fprintf('Calculating Final Lagrangian Hessian...\n');
    
    % Combine the cost and constraint hessians
    lagrangian_hessian_final = cost_hessian + constraint_hessian_final;
    
    % Only use the lower triangular part since it is symmetric
    lagrangian_hessian_full_final = lagrangian_hessian_final;
    lagrangian_hessian_final = simplify(tril(lagrangian_hessian_final));
    
    % Non zero entries in the lagrangian hessian
    lagrangian_hessian_final_nz = lagrangian_hessian_final(lagrangian_hessian_final~=0);
    
    fprintf('Calculating Final Lagrangian Hessian Sparsity Pattern...\n')
    
    % Find the non-zero row and column indices
    [row_index_final_H, col_index_final_H] = find(lagrangian_hessian_final);
    
    % Shift indices for C++
    row_index_final_H = row_index_final_H - 1;
    col_index_final_H = col_index_final_H - 1;
    
    % Add the iteration index modifier
    row_index_final_H = row_index_final_H +...
        ones(size(row_index_final_H,1),1)*iter*NUM_X;
    col_index_final_H = col_index_final_H +...
        ones(size(col_index_final_H,1),1)*iter*NUM_X;
    
    % Print out the non zero entry results
    fprintf('   Nonzeros in Lagrangian Hessian: %i\n',nnz(lagrangian_hessian));
    fprintf('   Nonzeros in final Lagrangian Hessian: %i\n',nnz(lagrangian_hessian_final));
    
    fprintf('DONE calculating Lagrangian Hessian: %f s\n\n',toc(hessian_timer));
    
end


%% Generate Functions
if GENERATE_FUNCTIONS
    functions_timer = tic;
    fprintf('Generating MATLAB functions...\n')
    
    if INITIALIZATION
        % Generate decision variable and constraint bounds
        fprintf('Bounds function Gen...\n')
        matlabFunction(X_lb, X_ub, constraint_ub, constraint_lb,...
            'file','PRMPC/MPCBounds',...
            'vars',{x0, x_d, r_hip, c_state, x_max, x_min, u_max, u_min});
        
        % Generate initial guess, reference policy, and footstep references
        fprintf('Initial Conditions function Gen...\n')
        matlabFunction(X0, u_ref0, p_ref,...
            'file','PRMPC/MPCInitialize',...
            'vars',{x0, x_d, r_hip, p_foot0, c_state, touchdown, x_touchdown, dt,...
            stance_fraction, flight_phase, T_s, m, I, g, K_ref});
    else
        % Generate cost function
        fprintf('Cost function Gen...\n');
        matlabFunction(J,...
            'file','PRMPC/MPCCost',...
            'vars',{x; u; x_d; u_ref; Q; R; dt; c_state; m; I; g});
        
        % Generate cost gradient function
        fprintf('Cost Gradient function Gen...\n');
        matlabFunction(cost_gradient,'file','PRMPC/MPCCostGradient',...
            'vars',{x; u; x_d; u_ref; Q; R; dt; c_state; m; I; g});
        
        % Generate constraints function
        fprintf('Constraints function Gen...\n');
        matlabFunction(constraints,...
            'file','PRMPC/MPCConstraints',...
            'vars',{x; u; xf; uf; dt; c_state; c_statef;...
            m; I; g; z_g; mu_g});
        
        % Generate final constraints function
        fprintf('Final Constraints function Gen...\n');
        matlabFunction(constraints_final,...
            'file','PRMPC/MPCConstraintsFinal',...
            'vars',{x; u; c_state; z_g; mu_g});
        
        % Generate constraint jacobian function
        fprintf('Constraint Jacobian function Gen...\n');
        matlabFunction(constraint_jacobian_nz,...
            'file','PRMPC/MPCConstraintJacobian',...
            'vars',{x; u; dt; c_state; c_statef; m; I; mu_g});
        
        % Generate constraint jacobian sparsity function
        fprintf('Constraint Jacobian Sparsity function Gen...\n')
        matlabFunction(row_index_CJ, col_index_CJ,...
            'file','PRMPC/MPCConstraintJacobianSP',...
            'vars', {iter, NUM_X, NUM_C});
        
        % Generate final constraint jacobian function
        fprintf('Final Constraint Jacobian function Gen...\n');
        matlabFunction(constraint_jacobian_final_nz,...
            'file','PRMPC/MPCConstraintJacobianFinal',...
            'vars',{c_state; mu_g});
        
        % Generate final constraint jacobian sparsity function
        fprintf('Final Constraint Jacobian Sparsity function Gen...\n')
        matlabFunction(row_index_final_CJ, col_index_final_CJ,...
            'file','PRMPC/MPCConstraintJacobianFinalSP',...
            'vars', {iter, NUM_X, NUM_C});
        
        % Generate lagrangian hessian function
        fprintf('Lagrangian Hessian function Gen...\n')
        matlabFunction(lagrangian_hessian_nz,...
            'file','PRMPC/MPCLagrangianHessian',...
            'vars',{x; u; x_d; Q; R; dt; c_state;...
            obj_factor; lambda; m; I});
        
        % Generate hessian sparsity function
        fprintf('Lagrangian Hessian Sparsity function Gen...\n')
        matlabFunction(row_index_H, col_index_H,...
            'file','PRMPC/MPCLagrangianHessianSP',...
            'vars', {iter, NUM_X});
        
        % Generate final lagrangian hessian function
        fprintf('Final Lagrangian Hessian function Gen...\n')
        matlabFunction(lagrangian_hessian_final_nz,...
            'file','PRMPC/MPCLagrangianHessianFinal',...
            'vars',{x; u; x_d; Q; R; dt; c_state;...
            obj_factor; m; I});
        
        % Generate final hessian sparsity function
        fprintf('Final Lagrangian Hessian Sparsity function Gen...\n')
        matlabFunction(row_index_final_H, col_index_final_H,...
            'file','PRMPC/MPCLagrangianHessianFinalSP',...
            'vars', {iter, NUM_X});
    end
    fprintf('DONE generating MATLAB functions: %f s\n\n',toc(functions_timer));
end

% Finalize generation statistics
fprintf(['DONE generating symbolic functions!\n',...
    '   Total time taken: %f s\n\n'],toc(SymbolicPRMPC_timer))