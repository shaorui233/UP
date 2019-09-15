% qdd = X(1:7,:);    % joint acceleration (includes floating base coordinates)
% qd  = X(8:14,:);   % joint velocity
% q   = X(15:21,:);  % joint position
% f_f = X(22:23,:);  % front foot force
% f_r = X(24:25,:);  % rear foot force
% tau_motor = X(26:29,:); % actuator torques

X_ic = Xs;