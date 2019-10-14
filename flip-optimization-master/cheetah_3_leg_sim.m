% add the spatial_v2 directory to your path first.

clear;
close all;


params = get_robot_params(0);
model = get_robot_leg_model(params);
rotor_model = get_robot_leg_rotor_model(params);

% simulation parameters
dt = 0.005;
N = 1000;
tf = N * dt;

% initial configuration
qi = [.2,.2];

% outputs
Q = zeros(2,N);  % positions
Q(:,1) = qi;
QD = zeros(2,N); % velocities
pf = zeros(3,N); % foot positions
anim = zeros(3 * 3, N); % animation points

% sim loop
for i = 1:N-1
    q = Q(:,i);
    [H,C,p0,J0,Jdqd,pf0,Jf0,Jdfqd0,vf] = dynamics_one_step(model,rotor_model,q,QD(:,i));
    
    % motor torques
    tau = [0, 0]';
    
    qdd = H \ (-C - tau);
    
    anim(:,i) = [pf0{1}; p0{2}; p0{1}];
    QD(:,i+1) = QD(:,i) + dt * qdd;
    Q(:,i+1) = Q(:,i) + dt * QD(:,i+1);
end

figure(888);

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
    drawnow;
    F(i) = getframe(888);
    end
end