
% add casadi library (this probably only works on Linux)
addpath(genpath('casadi'));

% add spatial v2 library (should work on all OS)
addpath(genpath('spatial_v2'));

[params,model] = get_3d_robot_model();
%% Contact schedule
% integration settings
res = 1; % change to increase/decrease timestep length without changing timing
N = 76 * res; % number of timesteps
dt = 0.01 / res; % time between timesteps
double_contact = 50*res; % timesteps spent with both feet on ground
single_contact = 0*res; % timesteps spent with single foot on ground
flight = N-double_contact-single_contact; % timesteps spent in the air
% build contact schedule (it needs to be size N+1 because of constraints)
cs = [2*ones(1,double_contact) ones(1,single_contact) zeros(1,flight) 0];


%% Initial conditions and integration settings
q_leg_init = .5 * [0 -pi*.8 1.5*pi];
q_init = [0 0 0 0 0 0 q_leg_init q_leg_init q_leg_init q_leg_init]';
zero_force = repmat({zeros(6,1)},4,1);
[~,~,~,~,~,pfi,~,~,~] = all_the_dynamics(model,q_init,zeros(18,1),zero_force,0);

%% Optimization Variables

opti = casadi.Opti();
X = opti.variable(18*3 + 12 + 12,N);

qdd  = X(1:18,:);
qd   = X(19:36,:);
q    = X(37:54,:);
f_fr = X(55:57,:);
f_fl = X(58:60,:);
f_rr = X(61:63,:);
f_rl = X(64:66,:);
tau_motor = X(67:78,:);

% other names
Q = [q;qd];

%% Cost Function
%nothing! (For now)...

mu = 0.7;

%% Constraints:
disp_box('Building Constraints');
tic;
Kd_j = diag([zeros(6,1); ones(12,1)]);

for k = 1:N-1

    disp([num2str(k) ' of ' num2str(N-1) ' cs: ' num2str(cs(k))]);
    qk = q(:,k); % q at timestep k
    qdk = qd(:,k);
    Qk = Q(:,k); % q,qd at timestep k
    qdjk = Qk((19+6):36); % qd for the joints at timestep k
    
    [Hek,Cek,~,~,~,pfek,Jfek,Jdfqdek,~] = casadi_compatible_dynamics(model,qk,qdk,zero_force,0);
    Hek = add_offboard_rotors(Hek,params.I_rot,1,[0 0 0 0 0 0 36 36 81 36 36 81 36 36 81 36 36 81]);
    
    J_stack = [Jfek{1};Jfek{2};Jfek{3};Jfek{4}];
    J_rear_stack = [Jfek{3};Jfek{4}];
    tau_d = .2 * Kd_j * qdk;
    
    if(cs(k+1) == 0)
        % constraints are just qdd, H
        Aek = Hek;
        xek = qdd(:,k);
        % ground reaction forces must be zero
        opti.subject_to(f_fr(:,k) == [0;0;0]);
        opti.subject_to(f_fl(:,k) == [0;0;0]);
        opti.subject_to(f_rr(:,k) == [0;0;0]);
        opti.subject_to(f_rl(:,k) == [0;0;0]);
    % if we have back feet on the ground
    elseif(cs(k+1) == 1)
        % front feet reaction forces must be zero
        opti.subject_to(f_fr(:,k) == [0;0;0]);
        opti.subject_to(f_fl(:,k) == [0;0;0]);
        % constraints now include rear reaction force and jacobian
        Aek = [Hek, J_rear_stack.'];
        xek = [qdd(:,k);f_rr(:,k);f_rl(:,k)];
        % rear feet reaction forces must be at least 30 N to prevent slip
        % TODO add back again.
        %opti.subject_to(f_r(2,k) <= -10);
    % if both feet on the ground
    else
        % constraints include both reaction forces and jacobian
        Aek = [Hek, J_stack.'];
        %opti.subject_to(f_f(2,k) <= -10);
        %opti.subject_to(f_r(2,k) <= -10);
        xek = [qdd(:,k);f_fr(:,k);f_fl(:,k);f_rr(:,k);f_rl(:,k)];
    end
    
    bek = [-Cek - tau_d + [0;0;0;0;0;0;tau_motor(:,k)]];
    
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
        opti.subject_to((pfek{3} - pfi{3}) == [0;0;0]);
        opti.subject_to((pfek{4} - pfi{4}) == [0;0;0]);
    end
    
    if(cs(k) >= 2)
        opti.subject_to((pfek{1} - pfi{1}) == [0;0;0]);
        opti.subject_to((pfek{2} - pfi{2}) == [0;0;0]);
        opti.subject_to((pfek{3} - pfi{3}) == [0;0;0]);
        opti.subject_to((pfek{4} - pfi{4}) == [0;0;0]);
    end
    
    
    % max torque
    % both feet on the ground, full torque all motors (TODO make more real)
    tmax = 38;
    hmax = 28;
    if(cs(k) == 2)
        opti.subject_to(tau_motor(:,k) <= hmax * ones(12,1));
        opti.subject_to(tau_motor(:,k) >= -hmax * ones(12,1));
        
    % rear feet on the ground, less torque on swing legs
    elseif(cs(k) == 1)
        opti.subject_to(tau_motor(:,k) <= hmax * ones(12,1));
        opti.subject_to(tau_motor(:,k) >= -hmax * ones(12,1));
        % needed to make the robot actually jump up.
        %opti.subject_to(Qk(2) >= 0.05);
    else
        % in flight, low torque limits
        opti.subject_to(tau_motor(:,k) <= hmax * ones(12,1));
        opti.subject_to(tau_motor(:,k) >= -hmax * ones(12,1));
    end
    
    % friction cone
%     opti.subject_to(f_f(1,k) <= - mu*f_f(2,k));
%     opti.subject_to(f_f(1,k) >= mu*f_f(2,k));
%     
%     opti.subject_to(f_r(1,k) <= - mu*f_r(2,k));
%     opti.subject_to(f_r(1,k) >= mu*f_r(2,k));   
    
    % joint velocity
    opti.subject_to(qdjk <= 18 * ones(12,1));
    opti.subject_to(qdjk >= -18 * ones(12,1));
end
toc;
% initial conditions

opti.subject_to(q(:,1) == q_init);    % inital configuration
opti.subject_to(qd(:,1) == zeros(18,1)); % initial velocity
opti.subject_to(q(6,N) == pi/2);
opti.subject_to(q(3,N) == 0);
% solve 
opti.set_initial(q,repmat(q_init,1,N));

disp_box('Starting IPOPT');
opti.solver('ipopt');
sol = opti.solve();

Xs = sol.value(X);
Qs = sol.value(Q);
taus = sol.value(tau_motor);


