%% ==========================Cheetah Body Plot=============================
% MIT Cheetah 2 Project
% Biomimetic Robotics Lab
% Gerardo Bledt
% January 19, 2016
%
% Function to plot a 3-dimensional rectangular shell representing the body
%of the Cheetah as defined by the Base Object of the Robot.

function PlotInitial(X0, r_hip, X1, X_lb, X_ub, p_ref, u_ref, s, cost, px, py)
NUM_FEET = 4;
NUM_STATES = 12;

% p_box = X0(1:3) + X_lb(13:15)

%
color = cell(4,1);
color{1} = ColorPicker('RED');
color{2} = ColorPicker('BLUE');
color{3} = 0.7*ColorPicker('GREEN');
color{4} = ColorPicker('HOKIE ORANGE');
color{5} = ColorPicker('BLACK');

% Transparency
alpha = 0.4;

% Line Width
LW{1} = 2;
LW{2} = 4;

% Marker Size
MS{1} = 50;
MS{2} = 35;

% Patch handle cells
h_p_foot = cell(4,1);

% Plot the initial CoM position
plot3(X0(1),X0(2),X0(3),'.',...
    'Color',0*[1;1;1],'MarkerSize',MS{1});

% Plot the next predicted CoM position
plot3(X1(1),X1(2),X1(3),'.',...
    'Color',0.1*[1;1;1],'MarkerSize',MS{2});

% Plot the next predicted CoM position
plot3([X0(1);X1(1)],[X0(2);X1(2)],[X0(3);X1(3)],'--',...
    'Color',0*[1;1;1],'LineWidth',LW{2});

% Scale the forces
f_scale = 1/100000000000;

% Rotate the hips into the world frame
RotM = QuaternionMatrix(RPYToQuaternion(X0(4:6)));
r_hip_box = [repmat([0;0;-0.1],4,1);repmat([0;0;0.1],4,1)]+[r_hip;r_hip];
r_body_world = repmat(X0(1:3,1),1,8) + RotM*reshape(r_hip_box,3,8);
r_hip_world = repmat(X0(1:3,1),1,4) + RotM*reshape(r_hip,3,4);

% Plot the robot body
vert = r_body_world';

% Setup box faces
faces = [1, 2, 4, 3;...  % bottom
    5, 6, 8, 7;...       % top
    1, 2, 6, 5;...       % front
    3, 4, 8, 7;...       % back
    1, 3, 7, 5;...       % right
    2, 4, 8, 6];         % left


% Setup patch handles
h_body = patch(1, 1, 1, color{5}, ...
    'FaceLighting', 'Gouraud', 'AmbientStrength', 0.75, ...
    'SpecularStrength', 0.9, 'SpecularExponent', 5, ...
    'BackFaceLighting','reverselit','FaceAlpha',alpha,...
    'EdgeColor',color{5},'LineWidth',LW{1});

% Plot the Faces of the box
set(h_body,'Vertices', vert, 'Faces', faces);

s = [1;0;0;1];
    
% Plot boxes for all bound boxes
for foot = 1:NUM_FEET
    n = NUM_STATES + 6*(foot - 1);
    if(s(foot))
    % Extract the 8 vertex points of the box
    vert_raw = [X_lb(n+1,1),X_ub(n+2,1),X_lb(n+3,1);... % RFB
        X_ub(n+1,1),X_ub(n+2,1),X_lb(n+3,1);...     % LFB
        X_lb(n+1,1),X_lb(n+2,1),X_lb(n+3,1);...     % RBB
        X_ub(n+1,1),X_lb(n+2,1),X_lb(n+3,1);...     % LBB
        X_lb(n+1,1),X_ub(n+2,1),X_ub(n+3,1);...     % RFT
        X_ub(n+1,1),X_ub(n+2,1),X_ub(n+3,1);...     % LFT
        X_lb(n+1,1),X_lb(n+2,1),X_ub(n+3,1);...     % RBT
        X_ub(n+1,1),X_lb(n+2,1),X_ub(n+3,1)];       % LBT
    vert = repmat(X0(1:3,1)',8,1) + vert_raw;
    
    % Setup patch handles
    h_p_foot{foot} = patch(1, 1, 1, color{foot}, ...
        'FaceLighting', 'Gouraud', 'AmbientStrength', 0.75, ...
        'SpecularStrength', 0.9, 'SpecularExponent', 5, ...
        'BackFaceLighting','reverselit','FaceAlpha',alpha,...
        'EdgeColor',color{foot},'LineWidth',LW{1});
    
    % Plot the Faces of the box
    set(h_p_foot{foot},'Vertices', vert, 'Faces', faces);
    end
    s(foot) = 1;
    if s(foot)
        vis = 'on';
        [~,y_ind]  = min(cost);
        [~,x_ind] = min(min(cost));
        if ((-1)^foot > 0)
            y_ind = size(y_ind,2) - y_ind;
        end
        
    else
        vis = 'off';
        x_ind = 1;
        y_ind = 1;
    end
    
%     p_ref(1,foot) = X0(1)+px(foot,x_ind);
%     p_ref(2,foot) = X0(2)+py(foot,y_ind(x_ind));
    
    % Plot the initial reference footstep location
    plot3(X0(1)+p_ref(1,foot),X0(2)+p_ref(2,foot),p_ref(3,foot),'.',...
        'Color',color{foot},'MarkerSize',MS{1},'Visible',vis);
    
    % Plot r_ref
    plot3([X0(1);X0(1)+p_ref(1,foot)],[X0(2);X0(2)+p_ref(2,foot)],[X0(3);p_ref(3,foot)],'--',...
        'Color',color{foot},'LineWidth',LW{2},'Visible',vis);
    
    % Plot foot vector from hip
%     plot3([r_hip_world(1,foot);X0(1)+X0(1)+px(foot,x_ind)],[r_hip_world(2,foot);X0(2)+py(foot,y_ind(x_ind))],[r_hip_world(3,foot);p_ref(3,foot)],'-',...
%         'Color',color{5},'LineWidth',2*LW{2},'Visible',vis);
    plot3([r_hip_world(1,foot);X0(1)+p_ref(1,foot)],[r_hip_world(2,foot);X0(2)+p_ref(2,foot)],[r_hip_world(3,foot);p_ref(3,foot)],'-',...
        'Color',color{5},'LineWidth',2*LW{2},'Visible',vis);
    
    % Plot f_ref
    plot3(X0(1)+[p_ref(1,foot);p_ref(1,foot)+f_scale*u_ref(n+4-NUM_STATES)],...
        X0(2)+[p_ref(2,foot);p_ref(2,foot)+f_scale*u_ref(n+5-NUM_STATES)],...
        [p_ref(3,foot);p_ref(3,foot)+f_scale*u_ref(n+6-NUM_STATES)],'-',...
        'Color',color{foot},'LineWidth',LW{2},'Visible',vis);
end