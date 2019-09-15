function [ model ] = get_robot_model( params )
% Get spatial_v2 style robot model

% coordinate systems:
% x forward, y left, z up

% bodies:
% 0   - fixed origin
% 1   - Px (translate horizontal) (massless)
% 2   - Pz (translate vertical)   (massless)
% 3   - Ry (pitch)                (base + ab/ad link, rotor)
% 4,5 - Front "leg"
% 6,7 - Rear "leg"


%% Various mass parameters and locations
I_body = mcI(params.body_mass, [0 0 0], params.i_body_mult * boxInertia(params.body_mass,[params.body_length, params.body_width, params.body_height]));
I_l1   = mcI(params.l1_mass, [0 0 -params.l1/2], boxInertia(params.l1_mass,[params.leg_rad*2 params.leg_rad*2 params.l1]));
I_l2   = mcI(params.l2_mass, [0 0 -params.l2/2], boxInertia(params.l2_mass,[params.leg_rad*2 params.leg_rad*2 params.l2]));

hip_x = [params.body_length -params.body_length]/2;

NLEGS = 2;

%% Initialize model struct:
model.NB = 7;                                  % number of bodies
model.NLEGS = NLEGS;                           % number of legs
model.gravity = [0 0 -9.81];                   % gravity
model.parent  = zeros(1,model.NB);             % parent body indices
model.jtype   = repmat({'  '},model.NB,1);     % joint types
model.Xtree   = repmat({eye(6)},model.NB,1);   % coordinate transforms
model.I       = repmat({zeros(6)},model.NB,1); % spatial inertias
model.Xfoot   = repmat({zeros(6)},NLEGS,1);    % feet
model.b_foot  = zeros(1,NLEGS);

nb = 0; % current body index

%% Base x translation (no mass)
nb = nb + 1;
model.parent(nb) = nb - 1;   % parent is previous
model.jtype{nb}  = 'Px';     % prismatic x
model.Xtree{nb}  = eye(6);   % on top of previous joint
model.I{nb}      = zeros(6); % massless

%% Base z translation (no mass)
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype{nb}  = 'Pz';
model.Xtree{nb}  = eye(6);
model.I{nb}      = zeros(6);

%% Base pitch (body mass)
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype{nb} = 'Ry';
model.Xtree{nb} = eye(6);
model.I{nb}     = I_body;

nb_base = nb;

%% Loop through legs (just two for now)
for leg = 1:NLEGS
    %% Hip
    nb = nb + 1;
    model.parent(nb) = nb_base;  % hip parent is base
    model.jtype{nb}  = 'Ry';     % rotate around y
    %                 flip so y+ is leg forward,   translate to front/back
    model.Xtree{nb}  = plux(rz(pi),[0 0 0]') * plux(eye(3),[hip_x(leg) 0 0]');
    model.I{nb}      = I_l1;
    
    %% Knee
    nb = nb + 1;
    model.parent(nb) = nb - 1; % knee parent is hip
    model.jtype{nb}  = 'Ry'; % rotate around y
    model.Xtree{nb}  = plux(eye(3),[0 0 -params.l1]');
    model.I{nb}      = I_l2;
    
    %% Foot (bonus)
    model.Xfoot{leg}  = plux(eye(3),[0 0 -params.l2]'); 
    model.b_foot(leg) = nb;
    
end

disp(['Created robot model with ' num2str(nb) ' coordinates!']);

end

% from Pat's model of cheetah 3
function I = boxInertia(mass, x)
    I = (norm(x)^2*eye(3) - diag(x.^2))*mass/12;
end

