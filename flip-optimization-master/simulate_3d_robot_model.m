clear;
close all;

[params,model] = get_3d_robot_model();

zero_force = repmat({zeros(6,1)},4,1);

% timing parameters
dt = 0.005;
N = 300;
tf = N*dt;

% initial joint configuration
Qi_body = zeros(6,1);
Qi_leg = [.2 -.1*pi .2*pi]';
Qi = [Qi_body; Qi_leg; Qi_leg; Qi_leg; Qi_leg];

[~,~,~,~,~,pfi,~,~,~] = all_the_dynamics(model,Qi,zeros(18,1),zero_force,0);


% simulation outputs
Q = zeros(18,N);
Q(:,1) = Qi;
QD = zeros(18,N);
pf = repmat({zeros(3,N)},4,1);
anim = zeros(3*16,N);

pf_a = [pfi{1}; pfi{2}; pfi{3}; pfi{4}];


% simulation loop
for i = 1:N-1
    q = Q(:,i);
    [H,C,p0,J0,Jdqd,pf0,Jf0,Jdfqd0,vf] = all_the_dynamics(model,q,QD(:,i),zero_force,0);
    H = add_offboard_rotors(H,params.I_rot,1,[0 0 0 0 0 0 36 36 81 36 36 81 36 36 81 36 36 81]);
    J_feet = [Jf0{1};Jf0{2};Jf0{3};Jf0{4}];
    Jdqd_feet = [Jdfqd0{1};Jdfqd0{2};Jdfqd0{3};Jdfqd0{4}];
    Kd_j = diag([zeros(6,1); ones(12,1)]);
    tau_d = .2 * Kd_j * QD(:,i);
    pf_c = [pf0{1};pf0{2};pf0{3};pf0{4}];
    spat_lin = [4 5 6];
    vf_c = [vf{1}(spat_lin);vf{2}(spat_lin);vf{3}(spat_lin);vf{4}(spat_lin)];
    k_stab = 100 * (pf_c - pf_a) + 20 * vf_c;
    
    % dynamics with constraints
    A = [H J_feet';
        J_feet zeros(12,12)];
    
    b = [-C - tau_d;
         -Jdqd_feet - k_stab];
     
     x = A \ b;
     
     qdd = x(1:18);
     f   = x(19:end);
     % fwd kinematics for animation
     anim(:,i) = [p0{7};p0{8};p0{9};pf0{1};
         p0{10};p0{11};p0{12};pf0{2};
         p0{13};p0{14};p0{15};pf0{3};
         p0{16};p0{17};p0{18};pf0{4}];
     
     % integrate
     QD(:,i+1) = QD(:,i) + dt * qdd;
    Q(:,i+1) = Q(:,i) + dt * QD(:,i+1);
end

figure(777);

a_pts_1 = [4 3 2 1 5 6 7 8];
a_pts_2 = a_pts_1 + 8;
a_pts_3 = [1 5 13 9 1];
for i = 1:N
    c1 = reshape(anim(:,i),3,[]);

    
    plot3(c1(1,a_pts_1),c1(2,a_pts_1),c1(3,a_pts_1),'-mo',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',10); hold on;
    plot3(c1(1,a_pts_2),c1(2,a_pts_2),c1(3,a_pts_2),'-mo',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',10);
    plot3(c1(1,a_pts_3),c1(2,a_pts_3),c1(3,a_pts_3),'-mo',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',10); hold off;
axis(.3 * [-2 2 -2 2 -4 1]);
    drawnow;
end