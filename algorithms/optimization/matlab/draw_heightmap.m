clear all
clc 
close all

color_list = { [ 0.5    0.5   0.5], ...
               [0    0.4470    0.7410], ...
               [0.9290    0.6940    0.1250], ...
               [0.8500    0.3250    0.0980], ...
               [0.4660    0.6740    0.1880], ...
               [0.3010    0.7450    0.9330], ...
               [0.6350    0.0780    0.1840]};
           
%% 
fn_path = '/home/dhkim/Repository/cheetah_test/optimization/matlab/functions';

addpath(fn_path)

data_path = '/home/dhkim/Repository/cheetah_test/optimization_data/2019-02-20-11_05_50';
iter = 0;
height_map = fn_read_file(data_path, 'HeightMap',121);
pos_file = sprintf('%d_body_pos',iter);
vel_file = sprintf('%d_body_vel',iter);
acc_file = sprintf('%d_body_acc',iter);
time_file = sprintf('%d_time',iter);
foot_file = sprintf('%d_foot_loc',iter);

body_pos = fn_read_file(data_path, pos_file,3);
body_vel = fn_read_file(data_path, vel_file,3);
body_acc = fn_read_file(data_path, acc_file,3);
time = fn_read_file(data_path, time_file,1);
foot_loc = fn_read_file(data_path, foot_file,12);

% sparse
for i = 0:60
    for j=0:160
        height_map_sparse(i+1,j+1) = height_map(2*i+1, 2*j+1);
    end
end
fig = fn_open_figures(5);

%% Draw Figure
figure(fig(1))
[x, y] = meshgrid(0:0.01:1.6, 0:0.01:0.6);
z = sin(x) + cos(y);
% surf(x, y, z)
mesh(x, y, height_map_sparse);
hold on
plot3(body_pos(1,:), body_pos(2,:), body_pos(3,:));

% foot step plot
j=1;
d = 0.2;
w = 0.12;
leg_length = zeros(4*size(foot_loc, 2),1);
for i = 1:size(foot_loc, 2)
    bx = body_pos(1,40*i+1);
    by = body_pos(2,40*i+1);
    bz = body_pos(3,40*i+1);
    plot3(bx,by,bz, 'o','linewidth',3, 'color', color_list{j});
    % body rectangle
    plot3([bx+d, bx+d], [by-w, by+w],[bz, bz], 'linewidth',1, 'color',color_list{j});
    plot3([bx+d, bx-d], [by-w, by-w],[bz, bz], 'linewidth',1, 'color',color_list{j});
    plot3([bx+d, bx-d], [by+w, by+w],[bz, bz], 'linewidth',1, 'color',color_list{j});
    plot3([bx-d, bx-d], [by-w, by+w],[bz, bz], 'linewidth',1, 'color',color_list{j});
    % end of body drawing
    if mod(i,2) == 1
        mark = 'o';
    else
        mark = 'x';
    end
    % FR
    plot3([bx+d, foot_loc(1,i)], [by-w, foot_loc(2,i)], [bz, foot_loc(3,i)], 'color', color_list{j});
    plot3(foot_loc(1,i), foot_loc(2,i), foot_loc(3,i), mark, 'color', color_list{j});
    fr_leg =  [foot_loc(1,i), foot_loc(2,i), foot_loc(3,i)] - [bx+d, by-w, bz];
    leg_length(4*i-3) = norm(fr_leg);
    
    % FL
    plot3([bx+d, foot_loc(4,i)], [by+w, foot_loc(5,i)], [bz, foot_loc(6,i)], 'color', color_list{j});
    plot3(foot_loc(4,i), foot_loc(5,i), foot_loc(6,i), mark, 'color', color_list{j});
    fl_leg = [foot_loc(4,i), foot_loc(5,i), foot_loc(6,i)] - [bx+d, by+w, bz] ;
    leg_length(4*i-2) = norm(fl_leg);
    
    % HR
    plot3([bx-d, foot_loc(7,i)], [by-w, foot_loc(8,i)], [bz, foot_loc(9,i)], 'color', color_list{j});
    plot3(foot_loc(7,i), foot_loc(8,i), foot_loc(9,i), mark, 'color', color_list{j});
    hr_leg = [foot_loc(7,i), foot_loc(8,i), foot_loc(9,i)] - [bx-d, by-w, bz];
    leg_length(4*i-1) = norm(hr_leg);
    
    % HL
    plot3([bx-d, foot_loc(10,i)], [by+w, foot_loc(11,i)], [bz, foot_loc(12,i)], 'color', color_list{j});
    plot3(foot_loc(10,i), foot_loc(11,i), foot_loc(12,i), mark, 'color', color_list{j});
    hl_leg = [foot_loc(10,i), foot_loc(11,i), foot_loc(12,i)] - [bx-d, by+w, bz];
    leg_length(4*i) = norm(hl_leg);
    
    j = j+1;
    if(j>7)
        j=1;
    end
end
%plot3(foot_loc(1,:), foot_loc(2,:), foot_loc(3,:),'linewidth',2)

colorbar
axis equal
hold off

figure(fig(5))
plot(leg_length)
length_cost = 0;
des_len = 0.25;
for i = 1:length(leg_length)
    length_cost = length_cost + (leg_length(i)-des_len) * (leg_length(i)-des_len);
end
length_cost

%% Body trajectory

figure(fig(2))
for i=1:3
    subplot(3,1,i)
    plot(time, body_pos(i,:));
end
xlabel('pos')

figure(fig(3))
for i=1:3
    subplot(3,1,i)
    plot(time, body_vel(i,:));
end
xlabel('vel')

figure(fig(4))
for i=1:3
    subplot(3,1,i)
    plot(time, body_acc(i,:));
end
xlabel('acc')
