function [ H,C,p,pf,Jf,Jdqdf,vf ] = get_simplified_dynamics( model )

% H(q) mass matrix
% C(q,qd) bias torques
% p(q) body locations
% pf(q) foot locations
% Jf(q) foot jacobians (not spatial)
% Jdqdf(q,qd) foot (not spatial)
% vf(q,qd) foot velocity

q = sym('q',[7 1],'real');
qd = sym('qd',[7 1],'real');
tau = sym('tau',[4 1],'real');
f_front = sym('ff',[3 1],'real');
f_front = [0; 0; 0; f_front];
f_rear  = sym('fr',[3 1],'real');
f_rear = [0; 0; 0; f_rear];

f = {f_front,f_rear};
tau = [0;0;0;tau];

[H,C,p,J,Jdqd,pf,Jf,Jdqdf,vf] = all_the_dynamics(model,q,qd,f,1);

D_cell = { {H,C},p,pf,Jf,Jdqdf,vf};

D_cell = simplify_dynamics(D_cell,1);
HC = D_cell{1};
H = HC{1};
C = HC{2};
p = D_cell{2};
pf = D_cell{3};
Jf = D_cell{4};
Jdqdf = D_cell{5};
vf = D_cell{6};
end

