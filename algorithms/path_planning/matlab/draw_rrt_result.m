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
fn_path = '/home/dhkim/Repository/Cheetah-Software/algorithms/path_planning/matlab/functions';

addpath(fn_path)

data_path = '/home/dhkim/Repository/Cheetah-Software/algorithms/path_planning/planning_data';
iter = 0;
result_path = fn_read_file(data_path, 'final_path',7);
planning_result = fn_read_file(data_path, 'planning_result',7);


fig = fn_open_figures(2);

path = result_path(1:3,:);
path_len = length(path);
path_len
line_segment = {}
seg = 1;
pt = 1;
for i=1:length(planning_result)
%     line_segment{seg} = zeros(3,1);
    data = planning_result(1:3,i);
    line_segment{seg}(1:3, pt) = data;
    pt = pt+1;
    if(norm(data) < 0.0001)
        seg = seg+1;
        pt = 1;
    end
end
%% Draw Figure
figure(fig(1))
hold on
plot(path(1,:), path(2,:),'linewidth',2);

for i=1:path_len
    dx = 0.07*cos(path(3,i));
    dy = 0.07*sin(path(3,i));
    quiver(path(1,i),path(2,i),dx,dy,'linewidth',2)
end

figure(fig(2))
hold on
num_line = length(line_segment)
for i=1: 5000
    plot(line_segment{i}(1,:), line_segment{i}(2,:));
end
