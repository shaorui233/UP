close all
clc
clear all
%%
addpath('./functions')
%load('./../matlab_log/data_exp_run4.mat')
%load('./../matlab_log/data_exp_run5_freq.mat')
%load('./../matlab_log/data_exp_bounding.mat')
%load('./../matlab_log/sim_data.mat')
%load('./../matlab_log/exp_hallway.mat')
load('./../matlab_log/exp_treadmill3.mat')

%load('./../matlab_log/exp_3.mat')
fig = fn_open_figures(8);

%%
st_idx = 4000;
end_idx = length(wbc_lcm_data.lcm_timestamp);%-85000;
%end_idx = st_idx + 1000; %length(wbc_lcm_data.lcm_timestamp);%-85000;
%time = linspace(0, 1, length(wbc_lcm_data.lcm_timestamp));
time = wbc_lcm_data.lcm_timestamp;

figure(fig(1))
for i =1:12
    subplot(4,3,i)
hold on
plot(time(st_idx:end_idx), wbc_lcm_data.Fr(st_idx:end_idx,i))
plot(time(st_idx:end_idx), wbc_lcm_data.Fr_des(st_idx:end_idx,i))
grid on
axis tight
end
xlabel('Fr')

% foot
figure(fig(2))
for i =1:12
    subplot(4,3,i)
hold on
j = mod(i,3);
if j==0 
plot(time(st_idx:end_idx), wbc_lcm_data.foot_pos(st_idx:end_idx,i))
plot(time(st_idx:end_idx), wbc_lcm_data.foot_pos_cmd(st_idx:end_idx,i))
else
   plot(time(st_idx:end_idx), wbc_lcm_data.foot_pos(st_idx:end_idx,i) - wbc_lcm_data.body_pos(st_idx:end_idx,j))
plot(time(st_idx:end_idx), wbc_lcm_data.foot_pos_cmd(st_idx:end_idx,i) - wbc_lcm_data.body_pos(st_idx:end_idx,j)) 
end

grid on
axis tight
end
xlabel('Foot')

% JPos
figure(fig(3))
for i =1:12
    subplot(4, 3, i)
    hold on
    plot(time(st_idx:end_idx), wbc_lcm_data.jpos(st_idx:end_idx,i))
    plot(time(st_idx:end_idx), wbc_lcm_data.jpos_cmd(st_idx:end_idx,i))
    grid on
    axis tight
end
xlabel('JPos')

% Body pos
figure(fig(4))
for i = 1:3
    subplot(3,1,i)
    hold on
        plot(time(st_idx:end_idx), wbc_lcm_data.body_pos(st_idx:end_idx,i))
    plot(time(st_idx:end_idx), wbc_lcm_data.body_pos_cmd(st_idx:end_idx,i))
    axis tight
end

% Body vel
figure(fig(5))
for i = 1:3
    subplot(3,1,i)
    hold on
        plot(time(st_idx:end_idx), wbc_lcm_data.body_vel(st_idx:end_idx,i))
    plot(time(st_idx:end_idx), wbc_lcm_data.body_vel_cmd(st_idx:end_idx,i))
    axis tight
end
xlabel('Body Vel')

% Body ori
figure(fig(6))
for i = 1:4
    subplot(4,1,i)
    hold on
        plot(time(st_idx:end_idx), wbc_lcm_data.body_ori(st_idx:end_idx,i))
    plot(time(st_idx:end_idx), wbc_lcm_data.body_ori_cmd(st_idx:end_idx,i))
    axis tight
end

% Body Ang vel
figure(fig(7))
for i = 1:3
    subplot(3,1,i)
    hold on
        plot(hw_vectornav.w(:,i))
    axis tight
end
xlabel('body ang vel')

figure(fig(8))

for i = 1:3
    subplot(3,1,i)
    plot(state_estimator.rpy(:,i))
end
xlabel('RPY')
