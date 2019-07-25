%% Converts the GRF to Momentum rate of change
function hDot = NonlinearInput(x, u, s)

% Number of robot contact feet
NUM_FEET = 4;

% Rotation Matrix to transform torques
rpy = [1;1;1];  % activate roll, pitch, or yaw
R = simplify(QuaternionMatrixSym(RPYToQuaternion(rpy.*x(4:6))'));

% Set up symbolic CoM forces and torque vectors
f_com = sym(zeros(3, 1));
tau_com = sym(zeros(3, 1));

% Iterate through all of the robot legs
for foot = 1:NUM_FEET
    
    % Leg state number
    n = 6*(foot - 1);
    
    % Foot position vector from CoM (CURRENTLY ONLY FOR FLAT GROUND)
    r_foot = u(1 + n: 3 + n);
    
    % Foot ground reaction forces
    f_foot = u(4 + n:6 + n);
    
    % Summation of CoM forces
    f_com = f_com + s(foot)*f_foot;
    
    % Summation of CoM torques
    tau_com = tau_com + s(foot)*CrossProd(r_foot)*f_foot;
end

% Net wrench on the CoM
hDot = simplify([f_com; R'*tau_com]);
end