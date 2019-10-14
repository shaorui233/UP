function [ H,C,p,pf,Jf,Jdqdf,vf ] = get_simplified_3d_dynamics( model )
q = sym('q',[18 1],'real');
qd = sym('qd',[18 1],'real');
tau = sym('tau',[12 1],'real');

f_fl = sym('ffl',[3 1],'real');
f_fl = [0; 0; 0; f_fl];

f_fr = sym('ffr',[3 1],'real');
f_fr = [0; 0; 0; f_fr];

f_rl = sym('frl',[3 1],'real');
f_rl = [0; 0; 0; f_rl];

f_rr = sym('frr',[3 1],'real');
f_rr = [0; 0; 0; f_rr];

f = {f_fr, f_fl, f_rr, f_rl};
zero_force = repmat({zeros(6,1)},4,1);
tau = [0; 0; 0; 0; 0; 0; tau];

[H,C,p,~,~,pf,Jf,Jdqdf,vf] = all_the_dynamics(model,q,qd,zero_force,1);

% D_cell = { {H,C},p,pf,Jf,Jdqdf,vf};
% 
% D_cell = simplify_dynamics(D_cell,1);
% HC = D_cell{1};
% H = HC{1};
% C = HC{2};
% p = D_cell{2};
% pf = D_cell{3};
% Jf = D_cell{4};
% Jdqdf = D_cell{5};
% vf = D_cell{6};
end

