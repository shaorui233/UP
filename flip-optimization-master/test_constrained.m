clear;
close all;

addpath('spatial_v2');

% test of constrained dynamics
params = get_robot_params(0);
model = get_robot_model(params);
rotor_model = get_rotor_model(params);

% zero spatial force on feet.
zero_force = {zeros(6,1),zeros(6,1)};

% simulation parameters
dt = 0.005;
N = 1000;
tf = N*dt;

% initial joint configuration
%Qi = .5*[0 0 0 -pi/6 pi/3 -pi/6 pi/3]'; 
Qi = 3*[0 0 0 -.1*pi .2*pi -.1*pi .2*pi]';
% do forward kinematics to find initial foot positions pfi
[~,~,~,~,~,pfi,~,~,~] = all_the_dynamics(model,Qi,zeros(7,1),zero_force,0);

% simulation outputs
Q = zeros(7,N);
Q(:,1) = Qi;      % positions
QD = zeros(7,N);     % velocities
pf_f = zeros(3,N);  % foot positions, front
pf_r = zeros(3,N);  % foot positions, rear
anim = zeros(3*7,N); % points to draw in animation

% only care about x, z position of feet
pfi{1} = pfi{1}([1 3]);
pfi{2} = pfi{2}([1 3]);
pf_a = [pfi{1}; pfi{2}];

% desired joint position for joint PD controller:
qdes = -[0 0 0 -pi/6 pi/3 -pi/6 pi/3]'; 
Kp = 500 * diag([0 0 0 1 1 1 1]);
Kd = 10 * diag([0 0 0 1 1 1 1]);

max_torque = 1000000000;

% simulate
for i = 1:N-1
    q = Q(:,i);
    [H,C,p0,J0,Jdqd,pf0,Jf0,Jdfqd0,vf] = dynamics_one_step(model,rotor_model,q,QD(:,i));
    
    %I_rot_apparent = 200 * params.I_rot;
    
    %H = H + I_rot_apparent * diag([zeros(3,1); ones(4,1)]);
    %H = add_offboard_rotors(H,2*params.I_rot,9,[0 0 0 1 1 1 1]);
    
    % stack foot Jacobians, remove y coordinates
    J_feet = [Jf0{1};Jf0{2}];
    J_feet = J_feet([1 3 4 6],:);
   
    % stack foot Jdqd, remove y coordinates
    Jdqd_feet = [Jdfqd0{1};Jdfqd0{2}];
    Jdqd_feet = Jdqd_feet([1 3 4 6]);
    
    % joint damping (applied only to rotary joints)
    Kd_j = diag([0 0 0 1 1 1 1]);
    
    % joint damping torque
    tau_d = .2*Kd_j * QD(:,i);
    tau_d(tau_d > max_torque) = max_torque;
    tau_d(tau_d < -max_torque) = -max_torque;
    
    % joint position controller:
    %qdes = qdes + .01 * rand(7,1) - .01;
    tau_motor = Kp*(-qdes - q) + Kd*(-QD(:,i));
    
    
    % constraint stabilization
    pfc = [pf0{1}([1 3]); pf0{2}([1 3])];
    k_stab = 100*(pfc - pf_a) + 20*[vf{1}([4 6]);vf{2}([4 6])];

    % constrained dynamics
    A = [H     J_feet'; 
        J_feet zeros(2*2)];
    
    b = [-C - tau_d + 0*tau_motor; 
        -Jdqd_feet - 1*k_stab];
    
    x = A \ b;
    
    qdd = x(1:7);
    f   = x(8:end);
    
    % use forward kinematics results for animation
    anim(:,i) = [pf0{2};p0{7};p0{6};p0{6};p0{4};p0{5};pf0{1}];
    
    % integrate
    QD(:,i+1) = QD(:,i) + dt * qdd;
    Q(:,i+1) = Q(:,i) + dt * QD(:,i+1);
    pf_f(:,i) = pf0{1};
    pf_r(:,i) = pf0{2};
end

figure(888);
%v = VideoWriter('test1.avi');
%open(v);
F(N) = struct('cdata',[],'colormap',[]);
for i = 1:N
    if(mod(i,3) == 0)
        t = dt*i;
        
    ca = reshape(anim(:,i),3,[]);
    plot(ca(1,:),ca(3,:),'-mo',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',10); axis(.5*[-2 2 -3 1]);
    title(['t = ' num2str(t)]);
    %pause(.1);
    drawnow;
    F(i) = getframe(888);
    %writeVideo(v,F(i));
    end
end
%close(v);