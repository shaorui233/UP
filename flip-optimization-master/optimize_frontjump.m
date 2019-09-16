% Optimize front_jump 

% run 'clear' before running this script if you've changed the dynamics

% add casadi library (this probably only works on Linux)
addpath(genpath('casadi'));

% add spatial v2 library (should work on all OS)
addpath(genpath('spatial_v2'));

%% Symbolic Dynamics
% only do symbolic dynamics if we need to
if ~(exist('symbolic_done','var'))
    disp_box('Generate Symbolic Dynamics');
    tic;
    params = get_robot_params(1);  % robot masses, lengths...
    model  = get_robot_model(params); % spatial_v2 rigid body tree + feet
    [H,C,p,pf,Jf,Jdqdf,vf] = get_simplified_dynamics(model); % symbolic functions for dynamics
    addpath(genpath('dynamics_out'));
    write_dynamics_to_file(H,C,p,pf,Jf,Jdqdf,vf);
    toc;
    
    symbolic_done = 1;
end

%% Contact schedule
% integration settings
res = 1; % change to increase/decrease timestep length without changing timing
N = 85 * res; % number of timesteps 76
dt = 0.01 / res; % time between timesteps
double_contact = 24*res; % timesteps spent with both feet on ground 26
single_contact = 22*res; % timesteps spent with single foot on ground

% N = 95 * res; % number of timesteps 95
% dt = 0.01 / res; % time between timesteps
% double_contact = 28*res; % timesteps spent with both feet on ground 28
% single_contact = 26*res; % timesteps spent with single foot on ground 
flight = N-double_contact-single_contact; % timesteps spent in the air
% build contact schedule (it needs to be size N+1 because of constraints)
cs = [2*ones(1,double_contact) ones(1,single_contact) zeros(1,flight) 0];


%% Initial conditions and integration settings

% zero spatial force on both feet (used as input to dynamics)
zero_force = {zeros(6,1),zeros(6,1)};


% set initial joint position
q_init = .5*[0 0 0 -pi*.8 1.5*pi -pi*.8 1.5*pi]'; 
% find initial foot location using forward kinematics
[~,~,~,~,~,pfi,~,~,~] = all_the_dynamics(model,q_init,zeros(7,1),zero_force,0);
% only care about x,z position of feet.
pfi{1} = pfi{1}([1 3]);
pfi{2} = pfi{2}([1 3]);




%% Optimization Variables

% create optimization object
opti = casadi.Opti();
% create optimization variables
X = opti.variable(7*3 + 4 + 4,N);

% names for optimization variables
qdd = X(1:7,:);    % joint acceleration (includes floating base coordinates)
qd  = X(8:14,:);   % joint velocity
q   = X(15:21,:);  % joint position
f_f = X(22:23,:);  % front foot force
f_r = X(24:25,:);  % rear foot force
tau_motor = X(26:29,:); % actuator torques


% inputs to the symbolic dynamics
Q = [q;qd]; % Q is q, q_dot for all timesteps
F_0 = zeros(6,N);  % zero external force
C_in = [Q;F_0];    % input to the function that calculates C, external forces are in J'*f, not in C for this formulation!

%% Cost Function
% desired end position
q_end_joints = .5*[0 0 0 -pi/6 pi/3 -pi/6 pi/3]'; 
q_end_des = 0.5 * ([q_end_joints(4:7)] +  [-1.2566 2.3562 -1.2566 2.3562]');
% error in joints at last timestep (not including body coordinates)
qerr = q(4:7,N) - q_end_des;
% minimize error^2
opti.minimize(qerr'*qerr);

% friction of ground
mu = 0.7;


%% Constraints
disp_box('Building constraints');
tic;

% damping of joints
Kd_j = diag([0 0 0 1 1 1 1]);


% Loop through timesteps, applying constraints
for k = 1:N-1
    
    disp([num2str(k) ' of ' num2str(N-1) ' cs: ' num2str(cs(k))]);
    
    % the 'k' suffix indicates the value of the variable at the current
    % timestep
    qk = q(:,k); % q at timestep k
    Qk = Q(:,k); % q,qd at timestep k
    qdjk = Qk(11:14); % qd for the joints at timestep k
    
    pf1k = pf_sym1(qk); % position of front foot
    pf2k = pf_sym2(qk); % position of rear foot
    Hek = H_sym(qk);    % mass matrix
    % modify mass matrix to include rotors
    Hek = add_offboard_rotors(Hek,2*params.I_rot,1,[0 0 0 36 81 36 81]);  
    % bias torques
    Cek = C_sym(C_in(:,k));
    % jacobians
    J1ek = Jf_sym1(qk);
    J2ek = Jf_sym2(qk);
    % stack jacobians, remove y coordinate
    Jfek = [J1ek([1 3],:); J2ek([1 3],:)];
    % damping torque
    tau_d = .05*Kd_j * Qk(8:end); %changed from .2
    
    % build Aek, xek such that Aek*xek = bek:
    % notice that we use cs(k+1) here because the integration puts in a 
    % 1 timestep delay
    % if we're in flight...
    if(cs(k+1) == 0)
        % constraints are just qdd, H
        Aek = Hek;
        xek = qdd(:,k);
        % ground reaction forces must be zero
        opti.subject_to(f_f(:,k) == [0;0]);
        opti.subject_to(f_r(:,k) == [0;0]);
    % if we have back feet on the ground
    elseif(cs(k+1) == 1)
        % front feet reaction forces must be zero
        opti.subject_to(f_f(:,k) == [0;0]);
        % constraints now include rear reaction force and jacobian
        Aek = [Hek, J2ek([1 3],:).'];
        xek = [qdd(:,k);f_r(:,k)];
        % rear feet reaction forces must be at least 30 N to prevent slip
        opti.subject_to(f_r(2,k) <= -10);
    % if both feet on the ground
    else
        % constraints include both reaction forces and jacobian
        Aek = [Hek, Jfek.'];
        opti.subject_to(f_f(2,k) <= -10);
        opti.subject_to(f_r(2,k) <= -10);
        xek = [qdd(:,k);f_f(:,k);f_r(:,k)];
    end
    
    % bek is always torque (bias + damping + motor)
    % the body coordinates aren't actuated
    bek = [-Cek - tau_d + [0;0;0;tau_motor(:,k)]];
    
    % euler integration of qd,q
    qd_int = qd(:,k) + dt * qdd(:,k);
    q_int  = q(:,k)  + dt * qd_int;
    
    % integrated Q
    Q_int = [q_int;qd_int];
    
    % integration constraint
    Q_next = [q(:,k+1);qd(:,k+1)];
    opti.subject_to(Q_next == Q_int);
    
    % dynamics constraint
    opti.subject_to(Aek * xek == bek);
    
    % constraint feet to their initial position, if appropriate
    if(cs(k) >= 1)
        opti.subject_to((pf2k([1 3]) - pfi{2}) == [0;0]);
    end
    
    if(cs(k) >= 2)
        opti.subject_to((pf1k([1 3]) - pfi{1}) == [0;0]);
    end
    
    % max torque
    % both feet on the ground, full torque all motors
    tmax = 2*26;% 2 times becasue it isactually using 4 legs but in this optimisation only two joints 
    hmax = 2*18;
    if(cs(k) == 2)
        opti.subject_to(tau_motor(:,k) <= [hmax;tmax;hmax;tmax]);
        opti.subject_to(tau_motor(:,k) >= -[hmax;tmax;hmax;tmax]);
        
    % rear feet on the ground, less torque on swing legs
    elseif(cs(k) == 1)
        opti.subject_to(tau_motor(:,k) <= [10;10;hmax;tmax]);
        opti.subject_to(tau_motor(:,k) >= -[10;10;hmax;tmax]);
        % needed to make the robot actually jump up.
        opti.subject_to(Qk(2) >= 0.05);
    else
        % in flight, low torque limits
        opti.subject_to(tau_motor(:,k) <= [6;6;6;6]);
        opti.subject_to(tau_motor(:,k) >= -[6;6;6;6]);
    end
    
    % friction cone
    opti.subject_to(f_f(1,k) <= - mu*f_f(2,k));
    opti.subject_to(f_f(1,k) >= mu*f_f(2,k));
    
    opti.subject_to(f_r(1,k) <= - mu*f_r(2,k));
    opti.subject_to(f_r(1,k) >= mu*f_r(2,k));   
    
    % joint velocity
    %opti.subject_to(qdjk <= [18;18;18;18]);
    %opti.subject_to(qdjk >= -[18;18;18;18]);
    opti.subject_to(qdjk <= [40;26;40;26]); % hip knee hip knee
    opti.subject_to(qdjk >= -[40;26;40;26]);
end


% Initial/terminal constraints
opti.subject_to(q(:,1) == q_init);    % inital configuration
opti.subject_to(qd(:,1) == zeros(7,1)); % initial velocity
% opti.subject_to(q(3,N) == -1.9*pi); % flipped at the end
% opti.subject_to(q(2,N) <= .2); % end position
% opti.subject_to(q(2,N) >= 0.1); % end position
% opti.subject_to(q(1,N) <= -.3);  % end position


opti.subject_to(q(3,N) == 0); % flipped at the end
opti.subject_to(q(2,N) <= 1.5); % end position
opti.subject_to(q(2,N) >= 0.7); % end position
opti.subject_to(q(1,N) >= .8);  % end position
%opti.subject_to(q(4:7,N) == q_init(4:7));  % this is now in the cost
%functi
toc;

%% Initial guess
opti.set_initial(q,repmat(q_init,1,N));


%% Solve!
disp_box('Starting IPOPT');
opti.solver('ipopt');
sol = opti.solve();


Xs = sol.value(X);
Qs = sol.value(Q);
Ffs = sol.value(f_f);
Frs = sol.value(f_r);
taus = sol.value(tau_motor);

write_results_to_file(Qs, taus, Ffs, Frs, "front_jump_v3.dat", dt, N);
%%enable this line if you want to save the file to a data file to be used
%%on the robot