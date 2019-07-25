function color = ColorPicker(request)
%% ============================Color Picker================================
% MIT Cheetah 2 Project
% Biomimetic Robotics Lab
% Gerardo Bledt
% February 9, 2016
%
% A simple color picker that stores various commonly used colors and allows
% a user to request a desired color from a string or from the assigned
% numerical value.

% Allow for choice by number or color string
if (isnumeric(request))
    % Switch case structure for the numeric request
    switch request
        case 0
            color = uisetcolor; % custom
        case 1
            color = [1, 1, 1]; % white
        case 2
            color = [0, 0, 0]; % black
        case 3
            color = [1, 0, 0]; % red
        case 4
            color = [0, 1, 0]; % green
        case 5
            color = [0, 0, 1]; % blue
        case 6
            color = [1, 1, 0]; % yellow
        case 7
            color = [1, 0, 1]; % magenta
        case 8
            color = [0, 1, 1]; % cyan
        case 9
            color = [102, 0, 0]/255; % Hokie maroon
        case 10
            color = [255, 102, 0]/255; % Hokie orange
        case 11
            color = [163, 31, 52]/255; % MIT red
        case 12
            color = [138, 139, 140]/255; % MIT gray
        case 13
            color = [194, 192, 191]/255; % MIT light gray
        case 14
            color = [89, 48, 1]/255; % brown
        case 15
            color= [255, 165, 0]/255; % orange
    end
else
    % Switch case structure for the string request
    switch request
        case 'CUSTOM'
            color = uisetcolor; % custom
        case 'WHITE'
            color = [1, 1, 1]; % white
        case 'BLACK'
            color = [0, 0, 0]; % black
        case 'RED'
            color = [1, 0, 0]; % red
        case 'GREEN'
            color = [0, 1, 0]; % green
        case 'BLUE'
            color = [0, 0, 1]; % blue
        case 'YELLOW'
            color = [1, 1, 0]; % yellow
        case 'MAGENTA'
            color = [1, 0, 1]; % magenta
        case 'CYAN'
            color = [0, 1, 1]; % cyan
        case 'HOKIE MAROON'
            color = [102, 0, 0]/255; % Hokie maroon
        case 'HOKIE ORANGE'
            color = [255, 102, 0]/255; % Hokie orange
        case 'MIT RED'
            color = [163, 31, 52]/255; % MIT red
        case 'MIT GRAY'
            color = [138, 139, 140]/255; % MIT gray
        case 'MIT LIGHT GRAY'
            color = [194, 192, 191]/255; % MIT light gray
        case 'BROWN'
            color = [89, 48, 1]/255; % brown
        case 'ORANGE'
            color= [255, 165, 0]/255; % orange
    end
end